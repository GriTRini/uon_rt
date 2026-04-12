#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>

#include "../rt_control/model/model.hpp"
#include "../rt_control/trajectory/trajectory_generator.hpp"

namespace py = pybind11;
using namespace rt_control;
using namespace rt_control::model;
using namespace rt_control::trajectory;

PYBIND11_MODULE(rt_control_cpp_impl, m) {
    m.doc() = "uon robotics simulation-only library (Model & Trajectory)";

    // 1. Enums
    py::enum_<TrajState>(m, "TrajState")
        .value("STOP", TrajState::STOP)
        .value("STOPPING", TrajState::STOPPING)
        .value("TRAPJ", TrajState::TRAPJ)
        .value("ATTRJ", TrajState::ATTRJ)
        .value("ATTRL", TrajState::ATTRL)
        .value("PLAYJ", TrajState::PLAYJ).export_values();

    py::enum_<LinkID>(m, "LinkID")
        .value("BASE", LinkID::base)
        .value("LINK1", LinkID::link1)
        .value("LINK2", LinkID::link2)
        .value("LINK3", LinkID::link3)
        .value("LINK4", LinkID::link4)
        .value("LINK5", LinkID::link5)
        .value("LINK6", LinkID::link6).export_values();

    // 2. RobotModel & Joint
    py::class_<Joint>(m, "Joint")
        .def_readwrite("id", &Joint::id)
        .def_property("origin", 
            [](const Joint &self) { return self.origin.matrix(); },
            [](Joint &self, const Eigen::Matrix4d &m) { self.origin = Eigen::Isometry3d(m); })
        .def_readwrite("axis", &Joint::axis);

    py::class_<RobotModel>(m, "RobotModel")
        .def(py::init<>())
        .def(py::init<const std::string&>())
        .def("load_model", &RobotModel::load_model)
        .def("get_model_name", &RobotModel::get_model_name)
        .def("get_joints", &RobotModel::get_joints)
        .def("get_min_angles", &RobotModel::get_min_angles)
        .def("get_max_angles", &RobotModel::get_max_angles)
        .def("get_max_angvels", &RobotModel::get_max_angvels)
        .def("get_max_angaccs", &RobotModel::get_max_angaccs)
        .def("forward_kinematics", [](const RobotModel& self, const angles_t& q) {
            return self.forward_kinematics(q).matrix(); 
        });

    // 3. TrajGenerator
    py::class_<TrajGenerator>(m, "TrajGenerator")
        .def(py::init<>())
        .def("initialize", &TrajGenerator::initialize)
        .def("update", &TrajGenerator::update)
        .def("stop", &TrajGenerator::stop)
        .def("trapj", &TrajGenerator::trapj, py::arg("goal_angles"), py::arg("goal_angvels") = angles_t::Zero())
        .def("attrl", py::overload_cast<const Eigen::Isometry3d&, const value_t&>(&TrajGenerator::attrl), py::arg("goal_tmat"), py::arg("kp") = 50.0)
        .def("attrl", py::overload_cast<value_t, value_t, value_t, value_t, value_t, value_t, value_t>(&TrajGenerator::attrl), 
             py::arg("x"), py::arg("y"), py::arg("z"), py::arg("r_deg"), py::arg("p_deg"), py::arg("yaw_deg"), py::arg("kp") = 50.0)
        .def("align_tcp_to_floor", &TrajGenerator::align_tcp_to_floor, py::arg("yaw_deg") = 0.0, py::arg("kp") = 100.0)
        .def("align_tcp_to_front", &TrajGenerator::align_tcp_to_front, py::arg("kp") = 100.0) // 여기 뒤에 세미콜론이 없어야 합니다.
        .def("goal_reached", &TrajGenerator::goal_reached, // ← 여기에 '.'이 빠져있었을 겁니다.
             py::arg("angles_enorm_thold") = 2.0,
             py::arg("pos_enorm_thold") = 0.002,
             py::arg("rot_enorm_thold") = 3.0,
             py::arg("angvels_enorm_thold") = 4.0,
             py::arg("vel_enorm_thold") = 0.004,
             py::arg("w_enorm_thold") = 6.0)
        
        // Properties (속성들 역시 '.'으로 시작해야 체이닝이 유지됩니다)
        .def_property_readonly("angles", &TrajGenerator::angles)
        .def_property_readonly("angvels", &TrajGenerator::angvels)
        .def_property_readonly("tmat", [](const TrajGenerator& self) { return self.tmat().matrix(); })
        .def_property_readonly("flange_tmat", [](const TrajGenerator& self) { return self.flange_tmat().matrix(); })
        .def_property_readonly("jmat", &TrajGenerator::jmat)
        .def_property_readonly("a", &TrajGenerator::a);
}