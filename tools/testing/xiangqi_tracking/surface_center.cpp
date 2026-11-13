// Same-frame inverse-depth center with spatial support and split-fit agreement.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

// Same sample floor as SurfaceFit; not a physical sensor spec.
constexpr int kMinPoints = 50;
// Depth residual trim, metres: 1 mm floor and 3× median |residual| (same rule as TLS).
constexpr double kAbsResidualFloorM = 0.001;
constexpr double kMadScale = 3.0;
// Each ray-coordinate quadrant around the center ray; count floor, not a fraction.
constexpr int kMinQuadrantPoints = 50;
// Minor-axis RMS of d-scaled (u,v), metres. 3 mm rejects slivers; a ~25 mm piece is far above.
constexpr double kMinSpreadM = 0.003;
// Inliers after three robust rounds, as a fraction of the caller-provided sample.
constexpr double kMinRetainedFraction = 0.6;
// Max 3D displacement vs leave-one-quadrant-out, metres. Empirical sensitivity, not iid SE.
// Misses axis-aligned 8–10 mm steps: each 3-quadrant subset still contains both heights.
constexpr double kMaxCenterSpreadM = 0.0015;
// Camera-Z RMS ceiling on retained samples, metres. Starting gross-nonplanarity gate.
// Accepts interleaved 6 mm two-plane clouds (RMS ~3 mm) and 8–10 mm steps (RMS 2.1–2.6 mm).
constexpr double kMaxDepthRmsM = 0.004;
// |n·(rx,ry,1)| floor, matching SurfaceFit's grazing test.
constexpr double kMinGrazingDenom = 0.2;
// Valid camera-Z interval, metres; same workspace as the caller pretrim.
constexpr double kZMinM = 0.02;
constexpr double kZMaxM = 0.6;
// Close inliers: |predicted-depth residual| < 1 mm over all n input points.
// Real 2880: min 0.154, p1 0.220, p5 0.278, median 0.434. Floor 0.20 rejects ~13 real
// samples and the interleaved 6 mm cloud (residuals ~3 mm). Does not catch staircases.
constexpr double kCloseResidualM = 0.001;
constexpr double kMinCloseFraction = 0.20;
// Opposite-half 3D center disagreement (metres) at 0/45/90/135° in d-scaled (u,v).
// Pairwise bound: two independent halves may differ by at most 4 mm, twice the
// nominal 2 mm positioning tolerance. This is a disagreement check, not a proof
// of absolute accuracy. The tighter 1.5 mm leave-quadrant-out gate also applies.
constexpr double kMaxHalfCenterSpreadM = 0.004;
constexpr int kOutputCount = 11;

double numpy_median(std::vector<double> values) {
    const int n = static_cast<int>(values.size());
    const auto middle = values.begin() + n / 2;
    std::nth_element(values.begin(), middle, values.end());
    const double upper = *middle;
    if (n % 2) return upper;
    const double lower = *std::max_element(values.begin(), middle);
    return 0.5 * (lower + upper);
}

bool finite_z(double z) {
    return std::isfinite(z) && kZMinM < z && z < kZMaxM;
}

int quadrant(double du, double dv) {
    const bool u_neg = du < 0.0;
    const bool v_neg = dv < 0.0;
    if (!u_neg && !v_neg) return 0;
    if (u_neg && !v_neg) return 1;
    if (u_neg && v_neg) return 2;
    return 3;
}

bool fit_coef(const Eigen::MatrixXd& A, const Eigen::VectorXd& y,
              const std::vector<int>& idx, Eigen::Vector3d& coef) {
    const int m = static_cast<int>(idx.size());
    if (m < 3) return false;
    // Centered sufficient statistics avoid allocating/refactoring an n-by-3
    // matrix for every spatial subset. Coordinates are already median-scaled.
    double sx=0.0, sy=0.0, st=0.0, sxx=0.0, sxy=0.0, syy=0.0, sxt=0.0, syt=0.0;
    for (int i : idx) {
        const double x=A(i,0), v=A(i,1), t=y(i);
        sx+=x; sy+=v; st+=t; sxx+=x*x; sxy+=x*v; syy+=v*v; sxt+=x*t; syt+=v*t;
    }
    const double xx=sxx-sx*sx/m, xy=sxy-sx*sy/m, yy=syy-sy*sy/m;
    const double xt=sxt-sx*st/m, yt=syt-sy*st/m;
    const double determinant=xx*yy-xy*xy, trace=xx+yy;
    // Reject collinear support before division; physical spread is checked below.
    if (!(determinant>1e-12*trace*trace) || !(trace>0.0)) return false;
    coef.x()=(xt*yy-yt*xy)/determinant;
    coef.y()=(yt*xx-xt*xy)/determinant;
    coef.z()=(st-coef.x()*sx-coef.y()*sy)/m;
    return coef.allFinite();
}

double predicted_z(double a_dot_coef, double d) {
    return d / (1.0 + a_dot_coef / d);
}

double center_z(const Eigen::Vector3d& coef, double d) {
    return d / (1.0 + coef.z() / d);
}

Eigen::Vector3d center_xyz(double z, double rx, double ry) {
    return Eigen::Vector3d(rx * z, ry * z, z);
}

}

extern "C" int robust_center_fit(const double* points, int n, const double* ray,
                                 double median, double* output) {
    for (int i = 0; i < kOutputCount; ++i)
        output[i] = std::numeric_limits<double>::quiet_NaN();
    if (n < kMinPoints) return 1;
    if (!std::isfinite(median) || !(kZMinM < median && median < kZMaxM)) return 1;
    if (!std::isfinite(ray[0]) || !std::isfinite(ray[1]) || !std::isfinite(ray[2])) return 1;
    const double rz = ray[2];
    if (std::abs(rz) < 1e-12) return 1;
    // Formulation assumes r_z=1; divide so (rx,ry) are ray coordinates in that frame.
    const double rx = ray[0] / rz;
    const double ry = ray[1] / rz;
    if (!std::isfinite(rx) || !std::isfinite(ry)) return 1;
    const double d = median;

    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd y(n);
    for (int i = 0; i < n; ++i) {
        const double xi = points[3 * i], yi = points[3 * i + 1], zi = points[3 * i + 2];
        if (!std::isfinite(xi) || !std::isfinite(yi) || !std::isfinite(zi) || !(zi > 0.0))
            return 1;
        const double u = xi / zi, v = yi / zi;
        A(i, 0) = d * (u - rx);
        A(i, 1) = d * (v - ry);
        A(i, 2) = 1.0;
        y(i) = d * (d / zi - 1.0);
    }

    std::vector<int> kept(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) kept[static_cast<std::size_t>(i)] = i;

    Eigen::Vector3d coef = Eigen::Vector3d::Zero();
    for (int round = 0; round < 3; ++round) {
        if (!fit_coef(A, y, kept, coef)) return 1;
        std::vector<double> abs_res(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            const double zhat = predicted_z(A.row(i).dot(coef), d);
            abs_res[static_cast<std::size_t>(i)] = std::abs(points[3 * i + 2] - zhat);
        }
        const double threshold = std::max(kAbsResidualFloorM, kMadScale * numpy_median(abs_res));
        std::vector<int> retained;
        retained.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i)
            if (abs_res[static_cast<std::size_t>(i)] < threshold) retained.push_back(i);
        if (static_cast<int>(retained.size()) < kMinPoints) return 1;
        kept.swap(retained);
    }
    if (!fit_coef(A, y, kept, coef)) return 1;

    const double zc = center_z(coef, d);
    if (!finite_z(zc)) return 3;
    const Eigen::Vector3d center = center_xyz(zc, rx, ry);

    Eigen::Vector3d normal(coef.x(), coef.y(),
                           1.0 - coef.x() * rx - coef.y() * ry + coef.z() / d);
    const double nrm = normal.norm();
    if (!std::isfinite(nrm) || nrm < 1e-15) return 1;
    normal /= nrm;
    if (normal.z() > 0.0) normal = -normal;
    const double grazing = normal.dot(Eigen::Vector3d(rx, ry, 1.0));
    if (std::abs(grazing) < kMinGrazingDenom) return 9;

    const int nkept = static_cast<int>(kept.size());
    const double retained_fraction = static_cast<double>(nkept) / static_cast<double>(n);
    if (retained_fraction < kMinRetainedFraction) return 6;

    int quad_count[4] = {0, 0, 0, 0};
    double mean0 = 0.0, mean1 = 0.0;
    for (int i : kept) {
        ++quad_count[quadrant(A(i, 0), A(i, 1))];
        mean0 += A(i, 0);
        mean1 += A(i, 1);
    }
    for (int q = 0; q < 4; ++q)
        if (quad_count[q] < kMinQuadrantPoints) return 4;
    mean0 /= nkept;
    mean1 /= nkept;
    double cov00 = 0.0, cov01 = 0.0, cov11 = 0.0;
    for (int i : kept) {
        const double e0 = A(i, 0) - mean0, e1 = A(i, 1) - mean1;
        cov00 += e0 * e0;
        cov01 += e0 * e1;
        cov11 += e1 * e1;
    }
    cov00 /= nkept;
    cov01 /= nkept;
    cov11 /= nkept;
    const double tr = cov00 + cov11;
    const double det = cov00 * cov11 - cov01 * cov01;
    const double disc = std::max(0.0, tr * tr - 4.0 * det);
    const double lmin = 0.5 * (tr - std::sqrt(disc));
    if (!(std::sqrt(std::max(0.0, lmin)) >= kMinSpreadM)) return 5;

    double depth_sumsq = 0.0, plane_sumsq = 0.0;
    int close = 0;
    std::vector<double> depth_res(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double zhat = predicted_z(A.row(i).dot(coef), d);
        const double r = points[3 * i + 2] - zhat;
        depth_res[static_cast<std::size_t>(i)] = r;
        if (std::isfinite(r) && std::abs(r) < kCloseResidualM) ++close;
    }
    for (int i : kept) {
        const double r = depth_res[static_cast<std::size_t>(i)];
        depth_sumsq += r * r;
        const Eigen::Vector3d p(points[3 * i], points[3 * i + 1], points[3 * i + 2]);
        const double pe = (p - center).dot(normal);
        plane_sumsq += pe * pe;
    }
    const double depth_rms = std::sqrt(depth_sumsq / nkept);
    const double plane_rms = std::sqrt(plane_sumsq / nkept);
    const double close_fraction = static_cast<double>(close) / static_cast<double>(n);
    output[6] = plane_rms;
    output[7] = depth_rms;
    output[9] = close_fraction;
    if (!std::isfinite(depth_rms) || depth_rms > kMaxDepthRmsM) return 8;
    if (close_fraction < kMinCloseFraction) return 10;

    double loqo_spread = 0.0;
    for (int q = 0; q < 4; ++q) {
        std::vector<int> subset;
        subset.reserve(static_cast<std::size_t>(nkept));
        for (int i : kept)
            if (quadrant(A(i, 0), A(i, 1)) != q) subset.push_back(i);
        Eigen::Vector3d cq;
        if (static_cast<int>(subset.size()) < kMinPoints || !fit_coef(A, y, subset, cq))
            return 7;
        const double zq = center_z(cq, d);
        if (!finite_z(zq)) return 7;
        loqo_spread = std::max(loqo_spread, (center_xyz(zq, rx, ry) - center).norm());
    }
    output[8] = loqo_spread;
    if (loqo_spread > kMaxCenterSpreadM) return 7;

    const double is2 = 0.7071067811865476;
    const double dirs[4][2] = {{1.0, 0.0}, {is2, is2}, {0.0, 1.0}, {-is2, is2}};
    double half_spread = 0.0;
    int splits = 0;
    for (int s = 0; s < 4; ++s) {
        std::vector<int> pos, neg;
        pos.reserve(static_cast<std::size_t>(nkept));
        neg.reserve(static_cast<std::size_t>(nkept));
        for (int i : kept) {
            if (dirs[s][0] * A(i, 0) + dirs[s][1] * A(i, 1) >= 0.0) pos.push_back(i);
            else neg.push_back(i);
        }
        if (static_cast<int>(pos.size()) < kMinPoints ||
            static_cast<int>(neg.size()) < kMinPoints) continue;
        Eigen::Vector3d cp, cn;
        if (!fit_coef(A, y, pos, cp) || !fit_coef(A, y, neg, cn)) return 11;
        const double zp = center_z(cp, d), zn = center_z(cn, d);
        if (!finite_z(zp) || !finite_z(zn)) return 11;
        half_spread = std::max(half_spread,
            (center_xyz(zp, rx, ry) - center_xyz(zn, rx, ry)).norm());
        ++splits;
    }
    if (splits == 0) return 11;
    output[10] = half_spread;
    if (half_spread > kMaxHalfCenterSpreadM) return 11;

    output[0] = center.x();
    output[1] = center.y();
    output[2] = center.z();
    output[3] = normal.x();
    output[4] = normal.y();
    output[5] = normal.z();
    output[6] = plane_rms;
    output[7] = depth_rms;
    output[8] = loqo_spread;
    output[9] = close_fraction;
    output[10] = half_spread;
    return 0;
}
