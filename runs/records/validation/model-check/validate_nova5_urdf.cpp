#include <iostream>
#include <urdf_parser/urdf_parser.h>

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    const auto model = urdf::parseURDFFile(argv[1]);
    if (!model) return 1;
    std::cout << "URDF parsed: " << model->links_.size() << " links, "
              << model->joints_.size() << " joints; root="
              << model->getRoot()->name << '\n';
    return model->links_.size() == 8 && model->joints_.size() == 7 ? 0 : 1;
}
