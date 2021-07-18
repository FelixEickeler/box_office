//
// Created by felix on 12.06.2021.
//
#include "helpers.h"

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

//auto helpers::split(const std::string &str, const char delim) {
//    std::vector<std::string> parts;
//    std::string part;
//    std::stringstream ss(str);
//    while (std::getline(ss, part, delim)) {
//        parts.push_back(part);
//    }
//    return parts;
//}

bool helpers::xyzc_compare(const XYZC &p1, const XYZC &p2) {
    // sort by class
    if (std::get<1>(p1) < std::get<1>(p2)) return true;
    else if (std::get<1>(p1) < std::get<1>(p2)) return false;

    if(std::get<0>(p1).x() > std::get<0>(p2).x()) return true;
    else if (std::get<0>(p1).x() < std::get<0>(p2).x()) return false;

    if(std::get<0>(p1).y() > std::get<0>(p2).y()) return true;
    else if (std::get<0>(p1).y() < std::get<0>(p2).y()) return false;

    if(std::get<0>(p1).z() > std::get<0>(p2).z()) return true;
    else if (std::get<0>(p1).z() < std::get<0>(p2).z()) return false;

    return false;
}

bool helpers::xyzc_objecttype_compare(const XYZC &xyzc, const XYZC &lookup) {
    return std::get<1>(xyzc) < std::get<1>(lookup);
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

