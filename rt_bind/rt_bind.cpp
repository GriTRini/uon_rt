#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "trajectory/rtb_trajectory_generator.hpp"

namespace py = pybind11;

PYBIND11_MODULE(rt_control_cpp_impl, m) {
    m.doc() = "uon robotics complete trajectory binding";

    py::enum_<rt_control::trajectory::TrajState>(m, "TrajState")
        .value("STOP", rt_control::trajectory::TrajState::STOP)
        .value("TRAPJ", rt_control::trajectory::TrajState::TRAPJ)
        .value("ATTRL", rt_control::trajectory::TrajState::ATTRL)
        .export_values();

    py::class_<rt_control::model::RobotModel>(m, "RobotModel")
        .def(py::init<const std::string&>());

    py::class_<rtb::trajectory::TrajGenerator>(m, "TrajGenerator")
        .def(py::init<>())
        .def("initialize", &rtb::trajectory::TrajGenerator::initialize_py)
        .def("update", &rtb::trajectory::TrajGenerator::update_py)
        .def("stop", &rtb::trajectory::TrajGenerator::stop_py)
        
        .def("trapj", &rtb::trajectory::TrajGenerator::trapj_py, py::arg("goal_q"), py::arg("goal_dq") = py::none())
        .def("attrl", &rtb::trajectory::TrajGenerator::attrl_py, py::arg("goal_tmat"), py::arg("kp") = 50.0)
        
        .def("goal_reached", &rtb::trajectory::TrajGenerator::goal_reached_py,
             py::arg("q_th") = py::none(), py::arg("p_th") = py::none(), py::arg("r_th") = py::none())
        
        .def_property_readonly("angles", &rtb::trajectory::TrajGenerator::angles_py)
        .def_property_readonly("angvels", &rtb::trajectory::TrajGenerator::angvels_py)
        .def_property_readonly("tmat", &rtb::trajectory::TrajGenerator::tmat_py)
        .def_property_readonly("jmat", &rtb::trajectory::TrajGenerator::jmat_py)
        .def_property_readonly("a", &rtb::trajectory::TrajGenerator::a_py);
}