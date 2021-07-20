//
// Created by felix on 12.06.2021.
//
#include "helpers.h"
//#include "pybind_helpers.h"


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

std::tuple<CGAL::Surface_mesh<Point>, std::array<Point, 4>> Plane2Mesh(Point o1, Point p1, Point p2, bool swap_normal_direction) {
    auto e0 = o1;
    auto e1 = p1 ;
    auto e2 = p2;
    auto _e3 = CGAL::cross_product(e1- e0,  e2- e0);
    _e3= (_e3 / CGAL::sqrt(_e3.squared_length()));
    auto e3 = e0 + _e3;
    std::array<Point, 4> plane_system{e0, e1, e2, e3};

    if(swap_normal_direction){
        _e3 *= -1;
    }
    //box thickness
    auto eth = (p1 - e0) * 0.001;
    std::array<Point , 8> obb_points;
    obb_points[0] = p2  - eth;
    obb_points[1] = o1  - eth;
    obb_points[2] = o1 - _e3 - eth;
    obb_points[3] = p2 - _e3 - eth;

    obb_points[4] = p2 - _e3 + eth;
    obb_points[5] = p2  + eth;
    obb_points[6] = o1  + eth;
    obb_points[7] = o1 - _e3 + eth;

    CGAL::Surface_mesh<Point> plane_mesh;
    auto box = CGAL::make_hexahedron(obb_points[0], obb_points[1], obb_points[2], obb_points[3],
                                     obb_points[4], obb_points[5], obb_points[6], obb_points[7],
                                     plane_mesh);
    return {plane_mesh, plane_system};

}