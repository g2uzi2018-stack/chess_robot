// Native translation of recognize_tracked.surface_position robust plane fit
// after points preparation. Not compiled or tested in this delivery.
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

double numpy_median(std::vector<double> values) {
    const int n = static_cast<int>(values.size());
    const auto middle = values.begin() + n / 2;
    std::nth_element(values.begin(), middle, values.end());
    const double upper = *middle;
    if (n % 2) return upper;
    const double lower = *std::max_element(values.begin(), middle);
    return 0.5 * (lower + upper);
}

Eigen::Vector3d xyz(const double* p, int i) {
    return Eigen::Vector3d(p[3 * i], p[3 * i + 1], p[3 * i + 2]);
}

}

extern "C" int robust_plane_fit(const double* points, int n, const double* ray,
                                double* output) {
    if (n < 50) return 1;

    std::vector<int> kept(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) kept[static_cast<std::size_t>(i)] = i;

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();

    for (int round = 0; round < 3; ++round) {
        mean.setZero();
        for (int i : kept) mean += xyz(points, i);
        mean /= static_cast<double>(kept.size());

        Eigen::Matrix3d gram = Eigen::Matrix3d::Zero();
        for (int i : kept) {
            const Eigen::Vector3d d = xyz(points, i) - mean;
            gram += d * d.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(gram);
        normal = solver.eigenvectors().col(0);

        std::vector<double> error(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i)
            error[static_cast<std::size_t>(i)] =
                std::abs((xyz(points, i) - mean).dot(normal));

        const double threshold = std::max(0.001, 3.0 * numpy_median(error));
        std::vector<int> retained;
        retained.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i)
            if (error[static_cast<std::size_t>(i)] < threshold) retained.push_back(i);
        if (static_cast<int>(retained.size()) < 50) return 1;
        kept.swap(retained);
    }

    double sumsq = 0.0;
    for (int i : kept) {
        const double e = (xyz(points, i) - mean).dot(normal);
        sumsq += e * e;
    }
    const double residual = std::sqrt(sumsq / static_cast<double>(kept.size()));
    output[6] = residual;

    const Eigen::Vector3d center_ray(ray[0], ray[1], ray[2]);
    const double denominator = normal.dot(center_ray);
    if (std::abs(denominator) < 0.2 || residual > 0.002) return 2;

    const double zcenter = normal.dot(mean) / denominator;
    if (!(0.02 < zcenter && zcenter < 0.6)) return 3;

    if (normal.z() > 0.0) normal = -normal;
    const Eigen::Vector3d center = center_ray * zcenter;
    output[0] = center.x();
    output[1] = center.y();
    output[2] = center.z();
    output[3] = normal.x();
    output[4] = normal.y();
    output[5] = normal.z();
    output[6] = residual;
    return 0;
}
