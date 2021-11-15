//
// Created by felix on 09.07.2021.
//

#ifndef BOXOFFICE_DATA_H
#define BOXOFFICE_DATA_H
#include "rasterizer.h"

std::array<boxy::Point, 8> Get_CubePoints();
std::array<boxy::Vector2f, 9> Get_Points_on_Plane();
std::array<boxy::Point, 9> Get_Diag_Cube_9_Samples();
std::tuple<std::array<boxy::Point, 9>, std::array<boxy::Vector2f,9>> Cube_Diag_Testpoints();

template<class TSplitStrategy>
std::tuple<mvbb::Discretization, std::vector<Point2D>> GenerateRasterizer3x3();

template<class TSplitStrategy>
std::tuple<mvbb::Discretization, std::vector<Point2D>> GenerateRasterizer3x7();

#include "data.ipp"

#endif //BOXOFFICE_DATA_H
