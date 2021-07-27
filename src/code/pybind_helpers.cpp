//
// Created by felix on 20.07.2021.
//

#include "helpers.h"
#include "pybind_helpers.h"

np_array helpers::xyzc_2_numpy(VectorView<pointcloud_xyzc::iterator>objr) {
    pointcloud_xyzc partial_cloud;
    std::copy(objr.begin(), objr.end(),back_inserter(partial_cloud));
    return xyzc_2_numpy(partial_cloud);
}

np_array helpers::xyzc_2_numpy(const pointcloud_xyzc &pointcloud) {

    //create numpy memory space
    auto shape = std::array<ssize_t, 2>{static_cast<ssize_t>(pointcloud.size()), 4};
    np_array ext_pointcloud(shape);

    // copy data to numpy array, Observation: <50m point cloud its not even worth to think about par_unseq
    auto ext_begin = ext_pointcloud.mutable_data();
    std::for_each(pointcloud.cbegin(), pointcloud.cend(),
                  [&point_cloud = std::as_const(pointcloud), &ext_begin = std::as_const(ext_begin)](
                          auto const &cgal_point) {
                      int idx = (&cgal_point - &point_cloud[0]) * 4;
                      ext_begin[idx + 0] = std::get<0>(cgal_point).x();
                      ext_begin[idx + 1] = std::get<0>(cgal_point).y();
                      ext_begin[idx + 2] = std::get<0>(cgal_point).z();
                      ext_begin[idx + 3] = float(std::get<1>(cgal_point));
                  });
    return ext_pointcloud;
}

np_array helpers::bbox_2_numpy(BBox bbox) {
    np_array numpy_bbox({8,3});
    auto nbm = numpy_bbox.mutable_data();

    auto cnt = 0;
    for(auto& point : bbox.get_vertices()){
        nbm[cnt + 0] = float(point.x());
        nbm[cnt + 1] = float(point.y());
        nbm[cnt + 2] = float(point.z());
        cnt+=3;
    }
    return numpy_bbox;
}

std::vector<Point> helpers::numpy_2_points(py::array_t<float> numpy83) {
    py::buffer_info buf1 = numpy83.request();
    auto *ptr1 = (float *) buf1.ptr;
    size_t rows = buf1.shape[0];
    size_t cols = buf1.shape[1];
    if(cols != 3){
        throw std::runtime_error("This numpy array must be Nx3; 8 for each vertex and 3 for x,y,z !");
    }

    std::vector<Point> tmp;
    for (size_t idx = 0; idx < rows; idx++)
        tmp.emplace_back(ptr1[idx*cols + 0], ptr1[idx*cols + 1], ptr1[idx*cols + 2]);
    return tmp;
}


pointcloud_xyzc helpers::numpy_2_xyzc(py::array_t<float> nx4) {
    py::buffer_info buf1 = nx4.request();
    auto *ptr1 = (float *) buf1.ptr;
    size_t rows = buf1.shape[0];
    size_t cols = buf1.shape[1];
    if(cols != 4){
        throw std::runtime_error("This numpy array must be Nx4; 3 for each vertex  x,y,z and one for the class!");
    }

    pointcloud_xyzc tmp;
    for (size_t idx = 0; idx < rows; idx++)
        tmp.emplace_back(std::make_tuple(Point(ptr1[idx*cols + 0], ptr1[idx*cols + 1], ptr1[idx*cols + 2]),  ptr1[idx*cols + 3]));
    return tmp;
}

Point helpers::numpy31_2_point(py::array_t<float> numpy31) {
    py::buffer_info buf1 = numpy31.request();
    auto *ptr1 = (float *) buf1.ptr;
    size_t rows = buf1.shape[0];
    size_t cols = buf1.shape[1];
    if(rows != 3 || cols != 1){
        throw std::runtime_error("This numpy array must be 3x1; 3 for x,y,z !");
    }

    return Point(ptr1[0], ptr1[1], ptr1[2]);
}