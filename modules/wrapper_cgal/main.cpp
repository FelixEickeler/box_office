#include "Evaluator.h"
#include "typedefs.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
namespace py = pybind11;

PYBIND11_MODULE(mvbb, module_handle){
    module_handle.doc() = "This module computes the MVBB Decomposition";

    py::class_<MvbbEvaluator>(module_handle, "MvbbEvaluator")
            .def(py::init<>())
            .def_property("pointcloud_path", &MvbbEvaluator::get_pointcloud_path, &MvbbEvaluator::set_pointcloud_path)
            .def_property("objectlist_path", &MvbbEvaluator::get_objectlist_path, &MvbbEvaluator::set_objectlist_path)
            .def("load_points", &MvbbEvaluator::load_points)
            .def("load_objects", &MvbbEvaluator::load_objects)
            .def("get_pointcloud", &MvbbEvaluator::get_pointcloud)
            .def("get_objects", &MvbbEvaluator::get_objectlist);
//            .def_property_readonly("image", [](MvbbEvaluator &self) {
//                py::array out = py::cast(self.make_image());
//                return out;
//            })
                    // .def("multiply_two", &SomeClass::multiply_two)
//            .def("multiply_two", [](SomeClass &self, float one, float two) {
//                return py::make_tuple(self.multiply(one), self.multiply(two));
//            })
//            .def("function_that_takes_a_while", &SomeClass::function_that_takes_a_while)
//            ;
}
//    handle.def("get_pointcloud", &get_pointcloud);
//    handle.def("get_pointcloud", &get_pointcloud, py::return_value_policy::move);


////#include <cgal>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <vector>
//#include <array>
//#include <iostream>
//#include <chrono>
//#include <execution>
//
//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
//typedef Kernel::Point_3 Point;
//typedef std::tuple<Point, int> XYZC;
//
//namespace py = pybind11;
//using np_array = py::array_t<float, py::array::c_style> ;
//
//
//
//auto get_pointcloud(){
//    std::vector<XYZC> point_cloud;
//    point_cloud.reserve(100000000);
//    for(int i = 0; i<100000000; ++i){
//        point_cloud.emplace_back(std::make_tuple(Point(i*1,i*2,i*3),i*4));
//    }
//
//
//    np_array ext_pointcloud({ 100000000, 4 });
//    auto ext_begin = ext_pointcloud.mutable_data();
//    std::for_each(point_cloud.cbegin(), point_cloud.cend(),
//        [&point_cloud = std::as_const(point_cloud), &ext_begin = std::as_const(ext_begin)](auto const& cgal_point) {
//            int idx = (&cgal_point - &point_cloud[0]) *4;
//            ext_begin[idx+0] = std::get<0>(cgal_point).x();
//            ext_begin[idx+1] = std::get<0>(cgal_point).y();
//            ext_begin[idx+2] = std::get<0>(cgal_point).z();
//            ext_begin[idx+3] = float(std::get<1>(cgal_point));
//        });
//
////    auto start = std::chrono::high_resolution_clock::now();
////    auto finish = std::chrono::high_resolution_clock::now();
////    std::chrono::duration<double> elapsed = finish - start;
////    std::cout << "Elapsed time: " << elapsed.count() << " s\n";
//      return ext_pointcloud;
//}
////

////}