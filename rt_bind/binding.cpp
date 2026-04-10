#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

// 상위 폴더의 헤더 참조
#include "rt_control/core/robot_factory.hpp"
#include "rt_control/core/robot_base.hpp"
#include "rt_control/trajectory/trajectory_generator.hpp"
#include "rt_control/model/model.hpp"

namespace py = pybind11;
using namespace rt_control;
using namespace rt_control::trajectory;

PYBIND11_MODULE(uon_rt, m) {
    m.doc() = "UON RT Control Complete Binding";

    // ==============================================================
    // 1. TrajState Enum
    // ==============================================================
    py::enum_<TrajState>(m, "TrajState")
        .value("STOP", TrajState::STOP)
        .value("STOPPING", TrajState::STOPPING)
        .value("TRAPJ", TrajState::TRAPJ)
        .value("ATTRJ", TrajState::ATTRJ)
        .value("ATTRL", TrajState::ATTRL)
        .value("PLAYJ", TrajState::PLAYJ)
        .export_values();

    // ==============================================================
    // 2. TrajGenerator (모든 기능 노출)
    // ==============================================================
    py::class_<TrajGenerator>(m, "TrajGenerator")
        .def(py::init<>())
        .def("initialize", &TrajGenerator::initialize)
        .def("update", &TrajGenerator::update, py::arg("dt"))
        
        // 🌟 수정: Isometry3d 입력을 Matrix4d로 받아서 변환
        .def("set_tcp_tmat", [](TrajGenerator& self, const Eigen::Matrix4d& mat) {
            Eigen::Isometry3d iso;
            iso.matrix() = mat;
            self.set_tcp_tmat(iso);
        })
        .def("set_tcp", &TrajGenerator::set_tcp, 
             py::arg("x"), py::arg("y"), py::arg("z"), py::arg("r_deg"), py::arg("p_deg"), py::arg("yaw_deg"))
        
        // 제어 명령
        .def("stop", &TrajGenerator::stop)
        .def("trapj", &TrajGenerator::trapj, py::arg("goal_angles"), py::arg("goal_angvels") = angles_t::Zero())
        
        // 🌟 수정: Isometry3d 입력을 Matrix4d로 받아서 변환
        .def("attrl", [](TrajGenerator& self, const Eigen::Matrix4d& mat, value_t kp) {
            Eigen::Isometry3d iso;
            iso.matrix() = mat;
            return self.attrl(iso, kp);
        }, py::arg("goal_tmat"), py::arg("kp") = 50.0)
        .def("attrl", py::overload_cast<value_t, value_t, value_t, value_t, value_t, value_t, value_t>(&TrajGenerator::attrl),
             py::arg("x"), py::arg("y"), py::arg("z"), py::arg("r_deg"), py::arg("p_deg"), py::arg("yaw_deg"), py::arg("kp") = 50.0)
        
        // 정렬 API
        .def("align_tcp_to_floor", &TrajGenerator::align_tcp_to_floor, py::arg("yaw_deg") = 0.0, py::arg("kp") = 100.0)
        .def("align_tcp_to_front", &TrajGenerator::align_tcp_to_front, py::arg("kp") = 100.0)
        
        // 정보 조회 (Getters)
        .def("angles", &TrajGenerator::angles)
        .def("angvels", &TrajGenerator::angvels)
        .def("angaccs", &TrajGenerator::angaccs)
        .def("jmat", &TrajGenerator::jmat)
        .def("a", &TrajGenerator::a)
        
        // 🌟 수정: Isometry3d 출력을 Matrix4d로 변환하여 리턴
        .def("tmat", [](const TrajGenerator& self) -> Eigen::Matrix4d { return self.tmat().matrix(); })
        .def("flange_tmat", [](const TrajGenerator& self) -> Eigen::Matrix4d { return self.flange_tmat().matrix(); })
        
        // 오차 및 목표 확인
        .def("angles_enorm", &TrajGenerator::angles_enorm)
        .def("pos_enorm", &TrajGenerator::pos_enorm)
        .def("rot_enorm", &TrajGenerator::rot_enorm)
        .def("goal_angles", &TrajGenerator::goal_angles)
        
        // 🌟 수정: optional<Isometry3d> 처리
        .def("goal_tmat", [](const TrajGenerator& self) -> std::optional<Eigen::Matrix4d> {
            auto res = self.goal_tmat();
            if (res) return res->matrix();
            return std::nullopt;
        })
        
        .def("goal_reached", &TrajGenerator::goal_reached, 
             py::arg("angles_enorm_thold") = 2.0, 
             py::arg("pos_enorm_thold") = 0.002, 
             py::arg("rot_enorm_thold") = 3.0,
             py::arg("angvels_enorm_thold") = 4.0, 
             py::arg("vel_enorm_thold") = 0.004, 
             py::arg("w_enorm_thold") = 6.0)
        
        // 🌟 수정: 리턴값 변환
        .def("solve_forward", [](const TrajGenerator& self, const angles_t& q) -> Eigen::Matrix4d {
            return self.solve_forward(q).matrix();
        });

    // ==============================================================
    // 3. RobotBase (모든 인터페이스 노출)
    // ==============================================================
    py::class_<RobotBase, std::unique_ptr<RobotBase>>(m, "RobotBase")
        // 수학 및 기구학 API
        .def("set_tcp", &RobotBase::set_tcp)
        
        // 🌟 수정: Isometry3d 출력 처리
        .def("get_current_pos", [](const RobotBase& self) -> Eigen::Matrix4d { return self.get_current_pos().matrix(); })
        .def("get_current_flange_pos", [](const RobotBase& self) -> Eigen::Matrix4d { return self.get_current_flange_pos().matrix(); })
        .def("get_jacobian", &RobotBase::get_jacobian)
        .def("get_task_vel", &RobotBase::get_task_vel)
        .def("solve_forward", [](const RobotBase& self, const angles_t& q) -> Eigen::Matrix4d { return self.solve_forward(q).matrix(); })

        // 궤적 제어 API
        .def("stop", &RobotBase::stop)
        .def("trapj", &RobotBase::trapj, py::arg("goal_angles"), py::arg("goal_angvels") = std::nullopt)
        
        // 🌟 수정: Isometry3d 입력 처리
        .def("attrl", [](RobotBase& self, const Eigen::Matrix4d& mat, value_t kp) {
            Eigen::Isometry3d iso;
            iso.matrix() = mat;
            return self.attrl(iso, kp);
        }, py::arg("goal_tmat"), py::arg("kp") = 50.0)
        
        .def("attrl", py::overload_cast<value_t, value_t, value_t, value_t, value_t, value_t, value_t>(&RobotBase::attrl),
             py::arg("x"), py::arg("y"), py::arg("z"), py::arg("r_deg"), py::arg("p_deg"), py::arg("yaw_deg"), py::arg("kp") = 50.0)
        .def("align_tcp_to_floor", &RobotBase::align_tcp_to_floor, py::arg("yaw_deg") = 0.0, py::arg("kp") = 100.0)
        .def("align_tcp_to_front", &RobotBase::align_tcp_to_front, py::arg("kp") = 100.0)
        .def("get_goal_reached", &RobotBase::get_goal_reached, py::arg("q_th") = std::nullopt, py::arg("p_th") = std::nullopt, py::arg("r_th") = std::nullopt)

        // 하드웨어 연결 API
        .def("open_connection", &RobotBase::open_connection, py::arg("ip") = "192.168.1.30", py::arg("port") = 12345)
        .def("close_connection", &RobotBase::close_connection)
        .def("connect_rt", &RobotBase::connect_rt, py::arg("ip") = "192.168.1.30", py::arg("port") = 12347)
        .def("disconnect_rt", &RobotBase::disconnect_rt)
        .def("servo_on", &RobotBase::servo_on)
        .def("servo_off", &RobotBase::servo_off)
        .def("get_current_angles", &RobotBase::get_current_angles)
        .def("get_current_angvels", &RobotBase::get_current_angvels)
        .def("set_digital_output", &RobotBase::set_digital_output);

    // ==============================================================
    // 4. RobotFactory & Model
    // ==============================================================
    py::class_<RobotFactory>(m, "RobotFactory")
        .def_static("create", &RobotFactory::create);

    py::class_<model::RobotModel>(m, "RobotModel")
        .def(py::init<const std::string&>());
}