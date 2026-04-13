#include <array>
#include <string>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "geofik.h"

namespace py = pybind11;

namespace {

template <size_t N>
std::array<double, N> array_from_numpy(const py::array_t<double, py::array::c_style | py::array::forcecast>& input,
                                       const std::string& name) {
    if (input.size() != static_cast<py::ssize_t>(N)) {
        throw py::value_error(name + " must contain exactly " + std::to_string(N) + " values");
    }

    std::array<double, N> output{};
    py::buffer_info buffer = input.request();
    const auto* data = static_cast<const double*>(buffer.ptr);
    for (size_t i = 0; i < N; ++i) {
        output[i] = data[i];
    }
    return output;
}

std::array<double, 9> rotation_from_numpy(const py::array_t<double, py::array::c_style | py::array::forcecast>& input) {
    if (input.ndim() == 1 && input.shape(0) == 9) {
        return array_from_numpy<9>(input, "rotation");
    }
    if (input.ndim() != 2 || input.shape(0) != 3 || input.shape(1) != 3) {
        throw py::value_error("rotation must have shape (3, 3) or (9,)");
    }

    std::array<double, 9> output{};
    auto unchecked = input.unchecked<2>();
    for (py::ssize_t row = 0; row < 3; ++row) {
        for (py::ssize_t col = 0; col < 3; ++col) {
            output[static_cast<size_t>(row * 3 + col)] = unchecked(row, col);
        }
    }
    return output;
}

py::array_t<double> qsols_to_numpy(const std::array<std::array<double, 7>, 8>& qsols) {
    py::array_t<double> output({8, 7});
    auto unchecked = output.mutable_unchecked<2>();
    for (py::ssize_t sol = 0; sol < 8; ++sol) {
        for (py::ssize_t joint = 0; joint < 7; ++joint) {
            unchecked(sol, joint) = qsols[static_cast<size_t>(sol)][static_cast<size_t>(joint)];
        }
    }
    return output;
}

py::array_t<double> jacobians_to_numpy(const std::array<std::array<std::array<double, 6>, 7>, 8>& jsols) {
    py::array_t<double> output({8, 7, 6});
    auto unchecked = output.mutable_unchecked<3>();
    for (py::ssize_t sol = 0; sol < 8; ++sol) {
        for (py::ssize_t joint = 0; joint < 7; ++joint) {
            for (py::ssize_t axis = 0; axis < 6; ++axis) {
                unchecked(sol, joint, axis) =
                    jsols[static_cast<size_t>(sol)][static_cast<size_t>(joint)][static_cast<size_t>(axis)];
            }
        }
    }
    return output;
}

py::array_t<double> matrix4_to_numpy(const Eigen::Matrix4d& matrix) {
    py::array_t<double> output({4, 4});
    auto unchecked = output.mutable_unchecked<2>();
    for (py::ssize_t row = 0; row < 4; ++row) {
        for (py::ssize_t col = 0; col < 4; ++col) {
            unchecked(row, col) = matrix(row, col);
        }
    }
    return output;
}

char first_char(const std::string& value, const std::string& name) {
    if (value.size() != 1) {
        throw py::value_error(name + " must be a single-character frame name");
    }
    return value[0];
}

py::tuple ik_q7(const py::array_t<double, py::array::c_style | py::array::forcecast>& r,
                const py::array_t<double, py::array::c_style | py::array::forcecast>& rotation,
                double q7,
                double q1_sing) {
    std::array<std::array<double, 7>, 8> qsols{};
    unsigned int nsols = franka_ik_q7(array_from_numpy<3>(r, "position"), rotation_from_numpy(rotation), q7, qsols, q1_sing);
    return py::make_tuple(nsols, qsols_to_numpy(qsols));
}

py::tuple ik_q6(const py::array_t<double, py::array::c_style | py::array::forcecast>& r,
                const py::array_t<double, py::array::c_style | py::array::forcecast>& rotation,
                double q6,
                double q1_sing,
                double q7_sing) {
    std::array<std::array<double, 7>, 8> qsols{};
    unsigned int nsols = franka_ik_q6(array_from_numpy<3>(r, "position"), rotation_from_numpy(rotation), q6, qsols, q1_sing, q7_sing);
    return py::make_tuple(nsols, qsols_to_numpy(qsols));
}

py::tuple ik_q4(const py::array_t<double, py::array::c_style | py::array::forcecast>& r,
                const py::array_t<double, py::array::c_style | py::array::forcecast>& rotation,
                double q4,
                double q1_sing,
                double q7_sing) {
    std::array<std::array<double, 7>, 8> qsols{};
    unsigned int nsols = franka_ik_q4(array_from_numpy<3>(r, "position"), rotation_from_numpy(rotation), q4, qsols, q1_sing, q7_sing);
    return py::make_tuple(nsols, qsols_to_numpy(qsols));
}

py::tuple ik_swivel(const py::array_t<double, py::array::c_style | py::array::forcecast>& r,
                    const py::array_t<double, py::array::c_style | py::array::forcecast>& rotation,
                    double theta,
                    double q1_sing,
                    unsigned int n_points,
                    unsigned int n_fine_search) {
    std::array<std::array<double, 7>, 8> qsols{};
    unsigned int nsols = franka_ik_swivel(
        array_from_numpy<3>(r, "position"), rotation_from_numpy(rotation), theta, qsols, q1_sing, n_points, n_fine_search);
    return py::make_tuple(nsols, qsols_to_numpy(qsols));
}

py::tuple j_ik_q7(const py::array_t<double, py::array::c_style | py::array::forcecast>& r,
                  const py::array_t<double, py::array::c_style | py::array::forcecast>& rotation,
                  double q7,
                  bool joint_angles,
                  const std::string& jacobian_ee,
                  double q1_sing) {
    std::array<std::array<std::array<double, 6>, 7>, 8> jsols{};
    std::array<std::array<double, 7>, 8> qsols{};
    unsigned int nsols = franka_J_ik_q7(array_from_numpy<3>(r, "position"), rotation_from_numpy(rotation), q7, jsols, qsols,
                                        joint_angles, first_char(jacobian_ee, "jacobian_ee"), q1_sing);
    return py::make_tuple(nsols, jacobians_to_numpy(jsols), qsols_to_numpy(qsols));
}

py::tuple j_ik_q6(const py::array_t<double, py::array::c_style | py::array::forcecast>& r,
                  const py::array_t<double, py::array::c_style | py::array::forcecast>& rotation,
                  double q6,
                  bool joint_angles,
                  const std::string& jacobian_ee,
                  double q1_sing,
                  double q7_sing) {
    std::array<std::array<std::array<double, 6>, 7>, 8> jsols{};
    std::array<std::array<double, 7>, 8> qsols{};
    unsigned int nsols = franka_J_ik_q6(array_from_numpy<3>(r, "position"), rotation_from_numpy(rotation), q6, jsols, qsols,
                                        joint_angles, first_char(jacobian_ee, "jacobian_ee"), q1_sing, q7_sing);
    return py::make_tuple(nsols, jacobians_to_numpy(jsols), qsols_to_numpy(qsols));
}

py::tuple j_ik_q4(const py::array_t<double, py::array::c_style | py::array::forcecast>& r,
                  const py::array_t<double, py::array::c_style | py::array::forcecast>& rotation,
                  double q4,
                  bool joint_angles,
                  const std::string& jacobian_ee,
                  double q1_sing,
                  double q7_sing) {
    std::array<std::array<std::array<double, 6>, 7>, 8> jsols{};
    std::array<std::array<double, 7>, 8> qsols{};
    unsigned int nsols = franka_J_ik_q4(array_from_numpy<3>(r, "position"), rotation_from_numpy(rotation), q4, jsols, qsols,
                                        joint_angles, first_char(jacobian_ee, "jacobian_ee"), q1_sing, q7_sing);
    return py::make_tuple(nsols, jacobians_to_numpy(jsols), qsols_to_numpy(qsols));
}

py::tuple j_ik_swivel(const py::array_t<double, py::array::c_style | py::array::forcecast>& r,
                      const py::array_t<double, py::array::c_style | py::array::forcecast>& rotation,
                      double theta,
                      bool joint_angles,
                      const std::string& jacobian_ee,
                      double q1_sing,
                      unsigned int n_points,
                      unsigned int n_fine_search) {
    std::array<std::array<std::array<double, 6>, 7>, 8> jsols{};
    std::array<std::array<double, 7>, 8> qsols{};
    unsigned int nsols = franka_J_ik_swivel(array_from_numpy<3>(r, "position"), rotation_from_numpy(rotation), theta, jsols,
                                            qsols, joint_angles, first_char(jacobian_ee, "jacobian_ee"), q1_sing,
                                            n_points, n_fine_search);
    return py::make_tuple(nsols, jacobians_to_numpy(jsols), qsols_to_numpy(qsols));
}

py::array_t<double> fk(const py::array_t<double, py::array::c_style | py::array::forcecast>& q, const std::string& ee) {
    return matrix4_to_numpy(franka_fk(array_from_numpy<7>(q, "q"), first_char(ee, "ee")));
}

py::array_t<double> jacobian_from_q(const py::array_t<double, py::array::c_style | py::array::forcecast>& q,
                                    const std::string& ee) {
    py::array_t<double> output({7, 6});
    auto jacobian = J_from_q(array_from_numpy<7>(q, "q"), first_char(ee, "ee"));
    auto unchecked = output.mutable_unchecked<2>();
    for (py::ssize_t joint = 0; joint < 7; ++joint) {
        for (py::ssize_t axis = 0; axis < 6; ++axis) {
            unchecked(joint, axis) = jacobian[static_cast<size_t>(joint)][static_cast<size_t>(axis)];
        }
    }
    return output;
}

double swivel(const py::array_t<double, py::array::c_style | py::array::forcecast>& q) {
    return franka_swivel(array_from_numpy<7>(q, "q"));
}

}  // namespace

PYBIND11_MODULE(_geofik, m) {
    m.doc() = "pybind11 bindings for GeoFIK, a geometric IK solver for the Franka arm";

    m.def("franka_ik_q7", &ik_q7, py::arg("position"), py::arg("rotation"), py::arg("q7"),
          py::arg("q1_sing") = PI / 2);
    m.def("franka_ik_q6", &ik_q6, py::arg("position"), py::arg("rotation"), py::arg("q6"),
          py::arg("q1_sing") = PI / 2, py::arg("q7_sing") = 0.0);
    m.def("franka_ik_q4", &ik_q4, py::arg("position"), py::arg("rotation"), py::arg("q4"),
          py::arg("q1_sing") = PI / 2, py::arg("q7_sing") = 0.0);
    m.def("franka_ik_swivel", &ik_swivel, py::arg("position"), py::arg("rotation"), py::arg("theta"),
          py::arg("q1_sing") = PI / 2, py::arg("n_points") = 500, py::arg("n_fine_search") = 3);
    m.def("franka_J_ik_q7", &j_ik_q7, py::arg("position"), py::arg("rotation"), py::arg("q7"),
          py::arg("joint_angles") = true, py::arg("jacobian_ee") = "E", py::arg("q1_sing") = PI / 2);
    m.def("franka_J_ik_q6", &j_ik_q6, py::arg("position"), py::arg("rotation"), py::arg("q6"),
          py::arg("joint_angles") = true, py::arg("jacobian_ee") = "E", py::arg("q1_sing") = PI / 2,
          py::arg("q7_sing") = 0.0);
    m.def("franka_J_ik_q4", &j_ik_q4, py::arg("position"), py::arg("rotation"), py::arg("q4"),
          py::arg("joint_angles") = true, py::arg("jacobian_ee") = "E", py::arg("q1_sing") = PI / 2,
          py::arg("q7_sing") = 0.0);
    m.def("franka_J_ik_swivel", &j_ik_swivel, py::arg("position"), py::arg("rotation"), py::arg("theta"),
          py::arg("joint_angles") = true, py::arg("jacobian_ee") = "E", py::arg("q1_sing") = PI / 2,
          py::arg("n_points") = 600, py::arg("n_fine_search") = 3);
    m.def("J_from_q", &jacobian_from_q, py::arg("q"), py::arg("ee") = "E");
    m.def("franka_fk", &fk, py::arg("q"), py::arg("ee") = "E");
    m.def("franka_swivel", &swivel, py::arg("q"));
}
