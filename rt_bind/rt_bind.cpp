#include <pybind11/pybind11.h>
#include "trajectory/rtb_trajectory_generator.hpp"

namespace py = pybind11;

PYBIND11_MODULE(rt_control_cpp_impl, m) {
    m.doc() = "uon robotics trajectory binding";

    // --- 1. TrajState Enum ---
    py::enum_<rt_control::trajectory::TrajState>(m, "TrajState")
        .value("STOP", rt_control::trajectory::TrajState::STOP)
        .value("STOPPING", rt_control::trajectory::TrajState::STOPPING)
        .value("TRAPJ", rt_control::trajectory::TrajState::TRAPJ)
        .value("ATTRL", rt_control::trajectory::TrajState::ATTRL)
        .export_values();

    // --- 2. RobotModel (초기화에 필수) ---
    py::class_<rt_control::model::RobotModel>(m, "RobotModel")
        .def(py::init<const std::string&>());

    // --- 3. TrajGenerator ---
    py::class_<rtb::trajectory::TrajGenerator>(m, "TrajGenerator")
        .def(py::init<>())
        .def("initialize", &rtb::trajectory::TrajGenerator::initialize_py)
        .def("update", &rtb::trajectory::TrajGenerator::update)
        .def("stop", &rtb::trajectory::TrajGenerator::stop)
        .def("trapj", &rtb::trajectory::TrajGenerator::trapj_py, 
             py::arg("goal_q"), py::arg("goal_dq") = py::none())
        .def("goal_reached", &rtb::trajectory::TrajGenerator::goal_reached_py,
             py::arg("q_th") = py::none(), py::arg("p_th") = py::none(), py::arg("r_th") = py::none())
        
        // dsrbind 스타일 Property 접근
        .def_property_readonly("angles", &rtb::trajectory::TrajGenerator::angles_py)
        .def_property_readonly("angvels", &rtb::trajectory::TrajGenerator::angvels_py)
        .def_property_readonly("tmat", &rtb::trajectory::TrajGenerator::tmat_py)
        .def_property_readonly("jmat", &rtb::trajectory::TrajGenerator::jmat_py);
}