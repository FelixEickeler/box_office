//
// Created by felix on 16.07.2021.
//

#include "helpers.h"
#include "helpers_nobind.h"

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