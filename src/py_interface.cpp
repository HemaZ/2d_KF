#include "kalman_filter.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

PYBIND11_MODULE(kalmanfilter, m) {
  m.doc() = "2D Position and Velocity Kalman Filter";
  py::class_<KalmanFilter>(m, "KalmanFilter")
      .def(py::init<double, double, double, double>())
      .def("Update", &KalmanFilter::Update)
      .def("GetX", &KalmanFilter::GetX,
           py::return_value_policy::reference_internal);
}