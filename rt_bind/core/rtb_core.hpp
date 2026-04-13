#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

// 사용자님이 만든 마스터 헤더 포함
#include "../../rt_control/rt.hpp"

namespace py = pybind11;

namespace rtb {

using value_t = rt_control::value_t;
using angles_t = rt_control::angles_t;

// [dsrb 스타일] Eigen -> NumPy 변환 헬퍼 (복사 방식)
template <typename Derived>
[[nodiscard]] inline py::array_t<double> to_pyarray(const Eigen::MatrixBase<Derived>& mat) {
    return py::cast(mat);
}

// Eigen Isometry3d -> NumPy array (4x4) 변환
[[nodiscard]] inline py::array_t<double> to_pyarray(const Eigen::Isometry3d& t) {
    return py::cast(t.matrix());
}

} // namespace rtb