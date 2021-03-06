//
// Created by felix on 08.10.21.
//

/*
*  Counding is little weird, plane_point() starts is reversed, and eigen starts 0,0 at top
*  x   0    0  | 0,1
*  0   x    0  | 1,2
*  x   x    x  | 0,2
* 0,2  1,2  2,2
 */
#include "data.h"
std::tuple <std::array<boxy::Point, 9>, std::array<boxy::Vector2f, 9>> Cube_Diag_Testpoints() {
    auto diag = Get_Diag_Cube_9_Samples();
    auto plane = Get_Points_on_Plane();
    return {diag, plane};
}

std::array<boxy::Vector2f, 9> Get_Points_on_Plane() {
    std::array<boxy::Vector2f, 9> pop = {{
             {2, 2 * sqrt(2)}, {2, sqrt(2)}, {2, 0},
             {1, 2 * sqrt(2)}, {1, sqrt(2)}, {1, 0},
             {0, 2 * sqrt(2)}, {0, sqrt(2)}, {0, 0},
     }};
    return pop;
}

std::array<boxy::Point, 9> Get_Diag_Cube_9_Samples() {
    std::array<boxy::Point, 9> dc9s = {{
               {0, 0, 0}, {1, 1, 0}, {2, 2, 0},
               {0, 0, 1}, {1, 1, 1}, {2, 2, 1},
               {0, 0, 2}, {1, 1, 2}, {2, 2, 2}
       }};
    return dc9s;
}

std::array<boxy::Point, 8> Get_CubePoints() {
    std::array<boxy::Point, 8> cube_points = {{
          {0,0,0}, {1,0,0}, {1,1,0},
          {0,1,0}, {0,1,1}, {0,0,1},
          {1,0,1}, {1,1,1},
  }};
    return cube_points;
}

