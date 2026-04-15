#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "trajectory/rtb_trajectory_generator.hpp"
#include "core/rtb_robot_base.hpp"
#include "../rt_control/model/model.hpp" 

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
        .def("open_connection", &rt_control::RobotBase::open_connection, 
             py::arg("ip") = "192.168.1.30", py::arg("port") = 12345)
        .def("close_connection", &rt_control::RobotBase::close_connection)
        .def("connect_rt", &rt_control::RobotBase::connect_rt, 
             py::arg("ip") = "192.168.1.30", py::arg("port") = 12347)
        .def("disconnect_rt", &rt_control::RobotBase::disconnect_rt)
        .def("servo_on", &rt_control::RobotBase::servo_on)
        .def("servo_off", &rt_control::RobotBase::servo_off)
        .def("set_digital_output", &rt_control::RobotBase::set_digital_output, 
             py::arg("index"), py::arg("value"))
        .def("set_tcp", &rt_control::RobotBase::set_tcp,
             py::arg("x"), py::arg("y"), py::arg("z"), py::arg("r_deg"), py::arg("p_deg"), py::arg("yaw_deg"))
        .def("stop", &rt_control::RobotBase::stop)
        
        .def("trapj", [](rt_control::RobotBase& self, const py::array_t<double>& goal_q) {
            Eigen::Map<const rt_control::angles_t, Eigen::Unaligned> m_q(goal_q.data());
            return self.trapj(m_q);
        }, py::arg("goal_q"))

        // 🌟 수정됨: Eigen::Ref를 사용하여 파이썬의 Row-Major 행렬을 C++ 레이아웃으로 자동 변환
        .def("attrl", [](rt_control::RobotBase& self, const Eigen::Ref<const Eigen::Matrix4d>& goal_tmat, double kp) {
            Eigen::Isometry3d target; 
            target.matrix() = goal_tmat; 
            return self.attrl(target, kp);
        }, py::arg("goal_tmat"), py::arg("kp") = 50.0)

        .def("align_to_floor", &rt_control::RobotBase::align_tcp_to_floor, 
             py::arg("yaw_deg") = 0.0, py::arg("kp") = 100.0)
        .def("goal_reached", [](rt_control::RobotBase& self, 
                                std::optional<double> q_th, std::optional<double> p_th, std::optional<double> r_th) {
            return self.get_goal_reached(q_th, p_th, r_th);
        }, py::arg("q_th") = py::none(), py::arg("p_th") = py::none(), py::arg("r_th") = py::none())
        
        .def_property_readonly("angles", [](rt_control::RobotBase& self) {
            auto q = self.get_current_angles();
            return q ? rtb::to_pyarray(*q) : py::cast<py::object>(py::none());
        })
        .def_property_readonly("tmat", [](rt_control::RobotBase& self) {
            return rtb::to_pyarray(self.get_current_pos());
        })
        .def_property_readonly("flange_tmat", [](rt_control::RobotBase& self) {
            return rtb::to_pyarray(self.get_current_flange_pos());
        });

    // 5. Factory
    m.def("create_robot", &rtb::robot::RobotFactoryAdapter::create, 
          py::arg("model_name"), "Factory to create a RobotBase instance");
}