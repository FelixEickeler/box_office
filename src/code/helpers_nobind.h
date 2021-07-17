//
// Created by felix on 16.07.2021.
//

#ifndef BOXOFFICE_HELPERS_NOBIND_H
#define BOXOFFICE_HELPERS_NOBIND_H

std::tuple<CGAL::Surface_mesh<boxy::Point>, std::array<boxy::Point,4>> Plane2Mesh(boxy::Point o1, boxy::Point p1, boxy::Point p2, bool swap_normal_direction=false);

#include <fstream>
#include <string>
#include <experimental/string>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/write_off_points.h>
#include "typedefs.h"
#include "trim.h"

#endif //BOXOFFICE_HELPERS_NOBIND_H
