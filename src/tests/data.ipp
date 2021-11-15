//
// Created by felix on 08.10.21.
//

#ifndef BOXOFFICE_DATA_IPP
#define BOXOFFICE_DATA_IPP
/*
*  Counding is little weird, plane_point() starts is reversed, and eigen starts 0,0 at top
*  x   0    0  | 0,1
*  0   x    0  | 1,2
*  x   x    x  | 0,2
* 0,2  1,2  2,2
 */
template<class TSplitStrategy>
std::tuple <mvbb::Discretization, std::vector<Point2D>> GenerateRasterizer3x3() {
    using namespace mvbb;
    auto arrp = Get_Points_on_Plane();
    std::vector<Point2D> plane_points(arrp.begin(), arrp.end());
    plane_points.erase(plane_points.cbegin() + 7);
    plane_points.erase(plane_points.cbegin() + 5);
    plane_points.erase(plane_points.cbegin() + 2);
    plane_points.erase(plane_points.cbegin() + 1);
    std::array<float,4> min_max = {0,0, 2, 2 * float(sqrt(2))};
    TSplitStrategy split_strategy;
    Discretization rasterizer(split_strategy, min_max, 3);
    for(auto& p2d : plane_points){
        rasterizer.insert(p2d);
    }
    return {rasterizer, plane_points};
}

/*  3 x 7
 *  0 0 0 0 1 1 1
 *  0 0 0 0 1 1 1
 *  0 1 1 1 1 1 0
 *
 *  1, 0, 0, 0, 0, 0,
 *  1, 0, 0, 0, 0, 0,
 *  1, 0, 0, 0, 0, 0,
 *  1, 0, 1, 0, 0, 1,
 *  1, 0, 1, 0, 0, 1,
 *  0, 0, 1, 0, 0, 1}
 *
 *  Rasterizer:
 *  Expected: true
 *   1 1 1 1 1 0
 *   0 0 0 0 0 0
 *   0 0 0 1 1 1
 *   0 0 0 0 0 0
 *   0 0 0 0 0 0
 *   0 0 0 1 1 1
 *
 */
template<class TSplitStrategy>
std::tuple <mvbb::Discretization, std::vector<Point2D>> GenerateRasterizer3x7() {
    using namespace mvbb;
    std::vector<Point2D> more_test_points = {{
             {1,0}, {2,0}, {3,0}, {4,0}, {5,0},
             {4,1}, {5,1}, {6,1},
             {4,2}, {5,2}, {6,2},
     }};

    //compensate as the grid with 0.5 as the points are defined in the middle
    std::array<float,4> min_max = {1,0, static_cast<float>(6), static_cast<float>(2)};
    TSplitStrategy split_strategy;
    Discretization rasterizer(split_strategy, min_max, 6);
    // again compensate
    for(auto& p2d : more_test_points){
        rasterizer.insert(p2d);
    }
    return {rasterizer, more_test_points};
}






#endif //BOXOFFICE_DATA_IPP


