//
// Created by felix on 09.06.2021.
//

#ifndef TEST_BIND_TYPEDEFS_H
#define TEST_BIND_TYPEDEFS_H
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <pybind11/numpy.h>

namespace boxy{
    typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    typedef Kernel::Point_3 Point;
    typedef std::tuple<Point, int> XYZC;
    typedef std::vector<XYZC> pointcloud_xyzc;

    namespace py = pybind11;
    using np_array = py::array_t<float, py::array::c_style> ;
    using objectlist = std::unordered_map<int, std::string>;

}

#endif //TEST_BIND_TYPEDEFS_H
