#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h> // 🌟 추가: std::chrono 시간 객체를 파이썬 datetime으로 변환

#include "trajectory/rtb_trajectory_generator.hpp"
#include "core/rtb_robot_base.hpp"
#include "../rt_control/model/model.hpp" 
// #include "../rt_control/core/robot_base.hpp" // 필요에 따라 실제 경로 반영

namespace py = pybind11;

PYBIND11_MODULE(rt_bind, m) {
    m.doc() = "uon robotics unified control library (Full Features & Keyword Arguments)";

    // 1. TrajState Enum
    py::enum_<rt_control::trajectory::TrajState>(m, "TrajState")
        .value("STOP", rt_control::trajectory::TrajState::STOP)
        .value("TRAPJ", rt_control::trajectory::TrajState::TRAPJ)
        .value("ATTRL", rt_control::trajectory::TrajState::ATTRL)
        .export_values();

    // 2. RobotModel
    py::class_<rt_control::model::RobotModel>(m, "RobotModel")
        .def(py::init<const std::string&>())
        .def("get_model_name", &rt_control::model::RobotModel::get_model_name);

    // 🌟 추가: RobotAlarm 구조체 바인딩
    py::class_<rt_control::RobotAlarm>(m, "RobotAlarm")
        .def_readonly("time", &rt_control::RobotAlarm::time)
        .def_readonly("level", &rt_control::RobotAlarm::level)
        .def_readonly("group", &rt_control::RobotAlarm::group)
        .def_readonly("index", &rt_control::RobotAlarm::index)
        .def_property_readonly("param", [](const rt_control::RobotAlarm& self) {
            // C++ 배열 param[3]을 파이썬 튜플로 깔끔하게 변환해서 반환
            return py::make_tuple(self.param[0], self.param[1], self.param[2]);
        });

    // 3. [Sim] TrajGenerator (시뮬레이션 용)
    py::class_<rtb::trajectory::TrajGenerator>(m, "TrajGenerator")
        .def(py::init<>())
        .def("initialize", &rtb::trajectory::TrajGenerator::initialize_py,
             py::arg("model"), py::arg("q"), py::arg("dq"), py::arg("ddq"))
        .def("update", &rtb::trajectory::TrajGenerator::update_py, py::arg("dt"))
        .def("stop", &rtb::trajectory::TrajGenerator::stop_py)
        .def("set_tcp", &rtb::trajectory::TrajGenerator::set_tcp_py, 
             py::arg("x"), py::arg("y"), py::arg("z"), py::arg("r"), py::arg("p"), py::arg("yaw"))
        .def("trapj", &rtb::trajectory::TrajGenerator::trapj_py, 
             py::arg("goal_q"), py::arg("goal_dq") = py::none())
        .def("attrl", &rtb::trajectory::TrajGenerator::attrl_py, 
             py::arg("goal_tmat"), py::arg("kp") = 50.0)
        .def("align_to_floor", &rtb::trajectory::TrajGenerator::align_tcp_to_floor_py, 
             py::arg("yaw_deg") = 0.0, py::arg("kp") = 100.0)
        .def("align_to_front", &rtb::trajectory::TrajGenerator::align_tcp_to_front_py, 
             py::arg("kp") = 100.0)
        .def("goal_reached", &rtb::trajectory::TrajGenerator::goal_reached_py,
             py::arg("q_th") = py::none(), py::arg("p_th") = py::none(), py::arg("r_th") = py::none())
        .def_property_readonly("angles", &rtb::trajectory::TrajGenerator::angles_py)
        .def_property_readonly("tmat", &rtb::trajectory::TrajGenerator::tmat_py)
        .def_property_readonly("flange_tmat", &rtb::trajectory::TrajGenerator::flange_tmat_py)
        .def_property_readonly("jmat", &rtb::trajectory::TrajGenerator::jmat_py);

    // 4. [Real] RobotBase (실제 하드웨어 용)
    py::class_<rt_control::RobotBase, std::unique_ptr<rt_control::RobotBase>>(m, "RobotBase")
        // 연결/통신
        .def("open_connection", &rt_control::RobotBase::open_connection, 
             py::arg("ip") = "", py::arg("port") = 0) // 기본값을 Base 선언에 맞춤
        .def("close_connection", &rt_control::RobotBase::close_connection)
        .def("connect_rt", &rt_control::RobotBase::connect_rt, 
             py::arg("ip") = "", py::arg("port") = 0)
        .def("disconnect_rt", &rt_control::RobotBase::disconnect_rt)
        
        // 구동 제어
        .def("servo_on", &rt_control::RobotBase::servo_on)
        .def("servo_off", &rt_control::RobotBase::servo_off)
        .def("set_digital_output", &rt_control::RobotBase::set_digital_output, 
             py::arg("index"), py::arg("value"))
        
        // 설정
        .def("set_tcp", &rt_control::RobotBase::set_tcp,
             py::arg("x"), py::arg("y"), py::arg("z"), py::arg("r_deg"), py::arg("p_deg"), py::arg("yaw_deg"))
        
        // 🌟 추가: Tool 설정 관련 바인딩
        .def("add_tool", &rt_control::RobotBase::add_tool,
             py::arg("name"), py::arg("weight"), py::arg("cog"), py::arg("inertia"))
        .def("set_tool", &rt_control::RobotBase::set_tool, py::arg("name"))
        .def("del_tool", &rt_control::RobotBase::del_tool, py::arg("name"))
        .def("change_collision_sensitivity", &rt_control::RobotBase::change_collision_sensitivity, py::arg("sensitivity"))

        // 🌟 추가: 알람(Error) 처리 바인딩
        .def("pop_alarm", &rt_control::RobotBase::pop_alarm)

        // 궤적 생성 및 제어
        .def("stop", &rt_control::RobotBase::stop)
        .def("trapj", [](rt_control::RobotBase& self, const py::array_t<double>& goal_q) {
            Eigen::Map<const rt_control::angles_t, Eigen::Unaligned> m_q(goal_q.data());
            return self.trapj(m_q);
        }, py::arg("goal_q"))

        // attrl (행렬 입력 방식)
        .def("attrl", [](rt_control::RobotBase& self, const Eigen::Ref<const Eigen::Matrix4d>& goal_tmat, double kp) {
            Eigen::Isometry3d target; 
            target.matrix() = goal_tmat; 
            return self.attrl(target, kp);
        }, py::arg("goal_tmat"), py::arg("kp") = 50.0)

        // 🌟 추가: attrl (오일러 각도 X, Y, Z, R, P, Yaw 입력 방식 오버로딩)
        .def("attrl", [](rt_control::RobotBase& self, double x, double y, double z, double r, double p, double yaw, double kp) {
            return self.attrl(x, y, z, r, p, yaw, kp);
        }, py::arg("x"), py::arg("y"), py::arg("z"), py::arg("r_deg"), py::arg("p_deg"), py::arg("yaw_deg"), py::arg("kp") = 50.0)

        // 정렬 및 상태 확인
        .def("align_to_floor", &rt_control::RobotBase::align_tcp_to_floor, 
             py::arg("yaw_deg") = 0.0, py::arg("kp") = 100.0)
        // 🌟 추가: 정면 정렬 바인딩
        .def("align_to_front", &rt_control::RobotBase::align_tcp_to_front, 
             py::arg("kp") = 100.0)
        .def("goal_reached", [](rt_control::RobotBase& self, 
                                std::optional<double> q_th, std::optional<double> p_th, std::optional<double> r_th) {
            return self.get_goal_reached(q_th, p_th, r_th);
        }, py::arg("q_th") = py::none(), py::arg("p_th") = py::none(), py::arg("r_th") = py::none())
        
        // 🌟 추가: Forward Kinematics 바인딩
        .def("solve_forward", [](rt_control::RobotBase& self, const py::array_t<double>& q) {
            Eigen::Map<const rt_control::angles_t, Eigen::Unaligned> m_q(q.data());
            return rtb::to_pyarray(self.solve_forward(m_q));
        }, py::arg("q"))

        // 읽기 전용 프로퍼티 (현재 상태)
        .def_property_readonly("angles", [](rt_control::RobotBase& self) {
            auto q = self.get_current_angles();
            return q ? rtb::to_pyarray(*q) : py::cast<py::object>(py::none());
        })
        // 🌟 추가: 현재 각속도 반환
        .def_property_readonly("angvels", [](rt_control::RobotBase& self) {
            auto dq = self.get_current_angvels();
            return dq ? rtb::to_pyarray(*dq) : py::cast<py::object>(py::none());
        })
        .def_property_readonly("tmat", [](rt_control::RobotBase& self) {
            return rtb::to_pyarray(self.get_current_pos());
        })
        .def_property_readonly("flange_tmat", [](rt_control::RobotBase& self) {
            return rtb::to_pyarray(self.get_current_flange_pos());
        })
        // 🌟 추가: 자코비안 행렬 반환
        .def_property_readonly("jmat", [](rt_control::RobotBase& self) {
            return rtb::to_pyarray(self.get_jacobian());
        })
        // 🌟 추가: Task Velocity 반환
        .def_property_readonly("task_vel", [](rt_control::RobotBase& self) {
            return rtb::to_pyarray(self.get_task_vel());
        });

    // 5. Factory
    m.def("create_robot", &rtb::robot::RobotFactoryAdapter::create, 
          py::arg("model_name"), "Factory to create a RobotBase instance");
}