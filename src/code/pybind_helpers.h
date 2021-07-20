//
// Created by felix on 20.07.2021.
//

#ifndef BOXOFFICE_PYBIND_HELPERS_H
#define BOXOFFICE_PYBIND_HELPERS_H
#include <fstream>
#include <string>
#include <experimental/string>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/write_off_points.h>
#include <pybind11/numpy.h>
#include "typedefs.h"
#include "trim.h"

namespace  boxy{
    namespace py = pybind11;
    using np_array = py::array_t<float, py::array::c_style> ;
}

namespace  helpers {

    boxy::np_array xyzc_2_numpy(boxy::VectorView<boxy::pointcloud_xyzc::iterator> objr);

// TODO fix to iterators to avoid copy before, also make these in a pybind file !
    np_array xyzc_2_numpy(const pointcloud_xyzc &pointcloud);

    np_array bbox_2_numpy(BBox bbox);

    std::vector<Point> numpy_2_points(py::array_t<float> numpy31);

    Point numpy31_2_point(py::array_t<float> numpy31);
}
#endif //BOXOFFICE_PYBIND_HELPERS_H
