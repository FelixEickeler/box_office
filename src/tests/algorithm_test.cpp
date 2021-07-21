//
// Created by felix on 09.07.2021.
//

#include "gtest/gtest.h"
#include "mvbb_algorithm.h"
#include <algorithm>
#include "typedefs.h"
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/write_off_points.h>
#include "data.h"
#include "rasterizer.h"
#include <string>
#include <iterator>

//TEST (algorithm_testing /*test suite name*/, FitAndSplitHirachy /*test name*/) {
//    {
//        using namespace mvbb;
//        using type_fash =  FitAndSplitNode<pointcloud_xyzc>;
//        FitAndSplitHierarchy<type_fash> fash;
//        pointcloud_xyzc abc;
//        type_fash k1(BBox{}, abc.cbegin(), abc.cend());
//        k1.volume = 100;
//        fash.flat_hierarchy.push_back(k1);
//        auto l1 =  fash.getNodes(0);
//        ASSERT_EQ((l1.begin())->volume, k1.volume);
//        type_fash k2(BBox{});
//        k2.volume = -1;
//        type_fash k3(BBox{});
//        k3.volume = -2;
//        fash.flat_hierarchy.push_back(k2);
//        fash.flat_hierarchy.push_back(k3);
//        auto front_l2 =  fash.getNodes(1).begin();
//        ASSERT_TRUE(front_l2->volume == k2.volume) << front_l2->volume;
//        ASSERT_TRUE(std::next(front_l2)->volume == k3.volume);
//        auto back_l2 =  fash.getNodes(1).end();
//        ASSERT_TRUE(std::next(back_l2,-1)->volume == k3.volume);
//        ASSERT_TRUE(std::next(back_l2,-2)->volume == k2.volume);
//
//
//        float vstart = -1000;
//        for(auto n = 0; n < 4; ++n){
//            auto insert = fash.flat_hierarchy.emplace_back(BBox{});
//            insert.volume = vstart;
//            vstart += vstart;
//        }
//
//        auto fwd_it = fash.getNodes(2).begin();
//        for(auto n = 0; n < 4; ++n){
//            ASSERT_TRUE(std::next(fwd_it,n)->volume == fash.flat_hierarchy[3+n].volume);
//        }
//
//        auto bck_it = fash.getNodes(2).end();
//        for(auto n = 0; n < 4; ++n){
//            ASSERT_TRUE(std::next(bck_it,-n)->volume == fash.flat_hierarchy[6-n].volume) << std::distance(fash.getNodes(2).begin(), fash.getNodes(2).end());
//        }
//
//
//
//    }
//}

TEST (algorithm_testing /*test suite name*/, Rasterizer_StepSize /*test name*/) {
    using namespace mvbb;
    std::array<float,4> min_max = {0,0, 2, 2 * float(sqrt(2))};
    Rasterizer<256> rasterizer(min_max);
    auto [step_x, step_y] = rasterizer.step_sizes();
    ASSERT_FLOAT_EQ(step_x, 0.0078431372549019607843137254902);
    ASSERT_FLOAT_EQ(step_y, 0.01109187107743603959844461744478);
}
/// TODO: Make Test not that idotic, it did not help...
TEST (algorithm_testing /*test suite name*/, Rasterizer_Insert /*test name*/) {
    using namespace mvbb;
    auto plane_points = Get_Points_on_Plane();
    std::array<float,4> min_max = {0,0, 2, 2 * float(sqrt(2))};
    Rasterizer<3> rasterizer(min_max);
    for(auto& p2d : plane_points){
        rasterizer.insert(p2d);
    }
    auto verity_matrix = Eigen::Matrix<uint32_t, 3, 3>::Ones();
    auto grid = rasterizer.get_grid();
    ASSERT_EQ(verity_matrix, grid);
}


/*
*  Counding is little weird, plane_point() starts is reversed, and eigen starts 0,0 at top
*  x   0    0  | 0,1
*  0   x    0  | 1,2
*  x   x    x  | 0,2
* 0,2  1,2  2,2
 */
std::tuple<mvbb::Rasterizer<3>, std::vector<Point2D>> GenerateRasterizer3x3(){

    using namespace mvbb;
    auto arrp = Get_Points_on_Plane();
    std::vector<Point2D> plane_points(arrp.begin(), arrp.end());
    plane_points.erase(plane_points.cbegin() + 7);
    plane_points.erase(plane_points.cbegin() + 5);
    plane_points.erase(plane_points.cbegin() + 2);
    plane_points.erase(plane_points.cbegin() + 1);
    std::array<float,4> min_max = {0,0, 2, 2 * float(sqrt(2))};
    Rasterizer<3> rasterizer(min_max);
    for(auto& p2d : plane_points){
        rasterizer.insert(p2d);
    }
    return {rasterizer, plane_points};
}

TEST (algorithm_testing /*test suite name*/, get_first_and_last_slot /*test name*/) {
    auto [rasterizer, _] = GenerateRasterizer3x3();
    const auto test_points_x = (MatrixXu(2,3) <<  0, 1, 2, 3, 3, 3).finished();
    auto res_x = rasterizer.get_first_and_last_slot<direction_ex>();
    ASSERT_EQ(test_points_x, res_x);

    const auto test_points_y = (MatrixXu(2,3) <<  0, 1, 0, 1, 2, 3).finished();
    auto res_y = rasterizer.get_first_and_last_slot<direction_ey>();
    ASSERT_EQ(test_points_y, res_y);
}


TEST (algorithm_testing /*test suite name*/, best_split_rasterized /*test name*/) {
    auto [rasterizer, _] = GenerateRasterizer3x3();
    auto msr_ex = rasterizer.best_split_rasterized<direction_ex>();
    auto msr_ey = rasterizer.best_split_rasterized<direction_ey>();
    ASSERT_EQ(msr_ex.index, 0);
    ASSERT_EQ(msr_ey.index, 0);
}


/*  3 x 7
 *  0 0 0 0 1 1 1
 *  0 0 0 0 1 1 1
 *  0 1 1 1 1 1 0
 *
 *  Rasterizer:
 *  Expected: true
 *   1 1 1 1 1 0
 *   0 0 0 0 0 0
 *   0 0 0 0 0 0
 *   0 0 0 1 1 1
 *   0 0 0 0 0 0
 *   0 0 0 1 1 1
 *
 */
std::tuple<mvbb::Rasterizer<6>, std::vector<Point2D>> GenerateRasterizer3x7(){
    using namespace mvbb;
    std::vector<Point2D> more_test_points = {{
            {1,0}, {2,0}, {3,0}, {4,0}, {5,0},
            {4,1}, {5,1}, {6,1},
            {4,2}, {5,2}, {6,2},
    }};
    std::array<float,4> min_max = {1,0, 6, 2};
    Rasterizer<6> rasterizer(min_max);
    for(auto& p2d : more_test_points){
        rasterizer.insert(p2d);
    }
    return {rasterizer, more_test_points};
}

TEST (algorithm_testing /*test suite name*/, get_first_and_last_slot2 /*test name*/) {
    auto [rasterizer, _] = GenerateRasterizer3x7();
    const auto test_points_x = (MatrixXu(2,6) <<  0, 0, 0, 0, 0, 3, 1,1,1,6,6,6).finished();
    auto res_x = rasterizer.get_first_and_last_slot<direction_ex>();
    ASSERT_EQ(test_points_x, res_x);
    const auto test_points_y = (MatrixXu(2,6) <<  0, 0, 0, 3, 0, 3, 5,0,0,6,0,6).finished();
    auto res_y = rasterizer.get_first_and_last_slot<direction_ey>();
    ASSERT_EQ(test_points_y, res_y);
}


TEST (algorithm_testing /*test suite name*/, min_split_rasterized2 /*test name*/) {
    using namespace mvbb;
    auto [rasterizer, _]  = GenerateRasterizer3x7();
    auto msr_ex = rasterizer.best_split_rasterized<direction_ex>();
    auto msr_ey = rasterizer.best_split_rasterized<direction_ey>();
    ASSERT_EQ(msr_ex.index, 2);
    ASSERT_EQ(msr_ex.area, 21);
    ASSERT_EQ(msr_ey.index, 0);
    ASSERT_EQ(msr_ey.area, 20);
}

/// Translate back to 3x7 grid;
TEST (algorithm_testing /*test suite name*/, min_split_2 /*test name*/) {
    using namespace mvbb;
    auto [rasterizer, _]  = GenerateRasterizer3x7();
    auto ms_ex = rasterizer.best_split<direction_ex>();
    auto ms_ey = rasterizer.best_split<direction_ey>();
    Vector2f ex_val;  ex_val << 3,0;
//    EXPECT_TRUE(false) << "Check this, as the res increases from the grid used, but should be fine in continuous space !" ;
    Vector2f ey_val;  ey_val << 0,0.4;
    ASSERT_EQ(ms_ex.begin_cut, ex_val);
    ASSERT_FLOAT_EQ(ms_ex.area, 8.4);
    ASSERT_EQ(ms_ey.begin_cut, ey_val);
    ASSERT_FLOAT_EQ(ms_ey.area, 8);
}

TEST (algorithm_testing /*test suite name*/, Algo_MVBB/*test name*/) {
    mvbb::Algo_MVBB<pointcloud_xyzc> algo;
    std::filesystem::path source_pig("../../tests/data/pig.off");
    std::filesystem::path proof_piggybox("../../tests/data/pig_bbox.off");
    CGAL::Surface_mesh<Point> sm;
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(source_pig, sm) || sm.is_empty()) {
        ASSERT_TRUE(false) << "Testfile not found.";
    }
    CGAL::Surface_mesh<Point> proof;
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(proof_piggybox, proof) || proof.is_empty()) {
        ASSERT_TRUE(false) << "Prooffile not found.";
    }

    pointcloud_xyzc points;
    for (const auto &v : vertices(sm)){
        points.push_back(std::make_tuple(sm.point(v), 0));
    }

    BBox proof_bbox;
    auto idx = 0;
    for(const auto& v : vertices(proof)) {
        proof_bbox.get_vertices()[idx] = proof.point(v);
        ++idx;
    }
    auto bbox = algo.fit_bounding_box(points);
    ASSERT_NEAR(bbox.volume(), proof_bbox.volume(), 0.01);
    }

TEST (algorithm_testing /*test suite name*/, Algo_PCA/*test name*/) {
    ASSERT_TRUE(false) << "Not implmented yet";
}

TEST (algorithm_testing /*test suite name*/, decompose3D/*test name*/) {
    mvbb::Algo_MVBB<pointcloud_xyzc> mvbb;
    std::filesystem::path source_pig("../../tests/data/bunny00.off");
    CGAL::Surface_mesh<Point> sm;
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(source_pig, sm) || sm.is_empty()) {
        ASSERT_TRUE(false) << "Testfile not found.";
    }
    pointcloud_xyzc points;
    for (const auto &v : vertices(sm)) {
        points.push_back(std::make_tuple(sm.point(v), 0));
    }

    uint32_t levels = 1;
    auto tree = mvbb::decompose3D(points, &mvbb, levels);
    for(int k =0; k <= levels+1; ++k){
        uint32_t sk = 0;
        for(auto& node : tree.getNodes(k)) {
            std::filesystem::path cell_path_name( "./bunny_tree_lvl" + std::to_string(k) + "_" + std::to_string(sk) + ".off");
            CGAL::Surface_mesh<Point> obb_sm;
            auto obb_points = node.bounding_box.get_vertices();
            auto box = CGAL::make_hexahedron(obb_points[0], obb_points[1], obb_points[2], obb_points[3],
                                             obb_points[4], obb_points[5], obb_points[6], obb_points[7], obb_sm);

            if (!CGAL::IO::write_OFF(cell_path_name, obb_sm, CGAL::parameters::stream_precision(6))) {
                ASSERT_TRUE(false) << "the tree was not written, not sure why, maybe check path: "
                                   << std::filesystem::current_path();
            }
            ++sk;
        }
    }
}






//
//TEST (algorithm_testing /*test suite name*/, Algo_MVBB /*test name*/) {
//
//}
