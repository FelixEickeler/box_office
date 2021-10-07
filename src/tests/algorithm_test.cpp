//
// Created by felix on 09.07.2021.
//

#include "gtest/gtest.h"
#include "mvbb_algorithms.h"
#include "typedefs.h"
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include "data.h"
#include "rasterizer.h"

TwoSplitStrategy two_split;

//TEST (algorithm_testing , FitAndSplitHirachy ) {
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

TEST (Discretization , Creation_Minmax_CorrectStepsize ) {
    using namespace mvbb;
    std::array<float,4> min_max = {0,0, 2, 2 * float(sqrt(2))};
    TwoSplitStrategy two_split;
    Discretization rasterizer(two_split, min_max, 256);
    auto [step_x, step_y] = rasterizer.step_sizes();
    ASSERT_FLOAT_EQ(step_x, 0.0078125);
    ASSERT_FLOAT_EQ(step_y, 0.011048543);
}

TEST (Discretization , Insert_Testpoints_CorrectGrid ) {
    using namespace mvbb;
    auto plane_points = Get_Points_on_Plane();
    std::array<float,4> min_max = {0,0, 2, 2 * float(sqrt(2))};
    TwoSplitStrategy two_split;
    MinMax2f mm(min_max);
    auto somesome = extend_boundaries(mm);
    Discretization rasterizer(two_split, min_max, 3);
    for(auto& p2d : plane_points){
        rasterizer.insert(p2d);
    }
    auto verity_matrix = Eigen::Matrix<uint32_t, 3, 3>::Ones();
    ASSERT_EQ(verity_matrix, rasterizer.Grid().data);
}


/*
*  Counding is little weird, plane_point() starts is reversed, and eigen starts 0,0 at top
*  x   0    0  | 0,1
*  0   x    0  | 1,2
*  x   x    x  | 0,2
* 0,2  1,2  2,2
 */
std::tuple<mvbb::Discretization, std::vector<Point2D>> GenerateRasterizer3x3(){

    using namespace mvbb;
    auto arrp = Get_Points_on_Plane();
    std::vector<Point2D> plane_points(arrp.begin(), arrp.end());
    plane_points.erase(plane_points.cbegin() + 7);
    plane_points.erase(plane_points.cbegin() + 5);
    plane_points.erase(plane_points.cbegin() + 2);
    plane_points.erase(plane_points.cbegin() + 1);
    std::array<float,4> min_max = {0,0, 2, 2 * float(sqrt(2))};
    Discretization rasterizer(two_split, min_max, 3);
    for(auto& p2d : plane_points){
        rasterizer.insert(p2d);
    }
    return {rasterizer, plane_points};
}

TEST (Discretization , GetFirstAndLastSlot_Testpoints3x3AndOrientation_CorrectSlotSpans ) {
    auto [rasterizer, _] = GenerateRasterizer3x3();
//    EXPECT_TRUE(false) << rasterizer.Grid().raster() << "\n";
    auto res_x = rasterizer.Grid().get_first_and_last_slot<GridOrientation::X>();
//    EXPECT_TRUE(false) << rasterizer.Grid().raster() << "\n";
    const auto test_points_x = (MatrixXu(2,3) <<  0, 1, 2, 3, 3, 3).finished(); // first 3 min, last 3 max
//    EXPECT_TRUE(false) << rasterizer.Grid().raster() << "\n";
    ASSERT_EQ(test_points_x, res_x);


    const auto test_points_y = (MatrixXu(2,3) <<  0, 1, 0, 1, 2, 3).finished();
    auto res_y = rasterizer.Grid().get_first_and_last_slot<GridOrientation::Y>();
    ASSERT_EQ(test_points_y, res_y);
}


TEST (Algorithm_TwoSplit , CalculateBestSplits_TestPoints3x3_CorrectGRIDSplit) {
    auto [rasterizer, _] = GenerateRasterizer3x3();
    auto [msr_ex, msr_ey] = two_split.calculate_best_splits(rasterizer.Grid());
    ASSERT_EQ(msr_ex.front().index, 0);
    ASSERT_EQ(msr_ey.front().index, 0);
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

std::tuple<mvbb::Discretization, std::vector<Point2D>> GenerateRasterizer3x7(){
    using namespace mvbb;
    std::vector<Point2D> more_test_points = {{
            {1,0}, {2,0}, {3,0}, {4,0}, {5,0},
            {4,1}, {5,1}, {6,1},
            {4,2}, {5,2}, {6,2},
    }};

    //compensate as the grid with 0.5 as the points are defined in the middle
    std::array<float,4> min_max = {1,0, static_cast<float>(6), static_cast<float>(2)};
    Discretization rasterizer(two_split, min_max, 6);
    // again compensate
    for(auto& p2d : more_test_points){
        rasterizer.insert(p2d);
    }
    return {rasterizer, more_test_points};
}

TEST (Algorithm_TwoSplit , GetFirstAndLastSlot_Testpoints3x7AndOrientation_CorrectSlotSpans ) {
    auto [rasterizer, _] = GenerateRasterizer3x7();
    const auto test_points_x = (MatrixXu(2,6) <<  0, 0, 0, 0, 0, 2, 1,1,1,6,6,6).finished();
    auto res_x = rasterizer.Grid().get_first_and_last_slot<GridOrientation::X>();
    ASSERT_EQ(test_points_x, res_x);
    const auto test_points_y = (MatrixXu(2,6) <<  0, 0, 3, 0, 0, 3, 5,0,6,0,0,6).finished();
    auto res_y = rasterizer.Grid().get_first_and_last_slot<GridOrientation::Y>();
    ASSERT_EQ(test_points_y, res_y);
}


TEST (Algorithm_TwoSplit , CalculateBestSplits_TestPoints3x7_CorrectGRIDSplit ) {
    using namespace mvbb;
    auto [rasterizer, _]  = GenerateRasterizer3x7();
    auto [msr_ex, msr_ey] = two_split.calculate_best_splits(rasterizer.Grid());
    ASSERT_EQ(msr_ex.front().index, 2);
    ASSERT_EQ(msr_ex.front().area, 21);
    ASSERT_EQ(msr_ey.front().index, 0);
    ASSERT_EQ(msr_ey.front().area, 20);
}

/// Translate back to 3x7 grid;
TEST (Algorithm_TwoSplit , CalculateBestSplits_TestPoints3x7_CorrectSPACESplit ) {
    using namespace mvbb;
    auto [rasterizer, _]  = GenerateRasterizer3x7();
    auto [ms_ex, ms_ey] = rasterizer.best_splits();
    ProjectedLine tx{{3.5,0},{3.5,2}};
    // perfect cut in x right of 3 => 3 < 3.5 (which is next, cut)
    // perfect cut in y at above 0.0 => 0.0 < 0.33  (which is next, cut)

    ProjectedLine ty{{1, 1.0/3},{6.0,1.0/3}};
    ASSERT_FLOAT_EQ(ms_ex.front().cut.start.x(), tx.start.x());
    ASSERT_FLOAT_EQ(ms_ex.front().cut.start.y(), tx.start.y());
    ASSERT_FLOAT_EQ(ms_ex.front().cut.end.x(), tx.end.x());
    ASSERT_FLOAT_EQ(ms_ex.front().cut.end.y(), tx.end.y());
    ASSERT_FLOAT_EQ(ms_ex.front().area, 21.0 * 5.0/6*1.0/3);

    ASSERT_FLOAT_EQ(ms_ey.front().cut.start.x(), ty.start.x());
    ASSERT_FLOAT_EQ(ms_ey.front().cut.start.y(), ty.start.y());
    ASSERT_FLOAT_EQ(ms_ey.front().cut.end.x(), ty.end.x());
    ASSERT_FLOAT_EQ(ms_ey.front().cut.end.y(), ty.end.y());
    ASSERT_FLOAT_EQ(ms_ex.front().area, 21.0 * 5.0/6*1.0/3);
}

TEST (Algorithm_MVBB , FitBoundingBox_Pig_PigBoxVolumeMatch) {
    mvbb::CGAL_MVBB<pointcloud_xyzc> algo;
    std::filesystem::path source_pig("./files/pig.off");
    std::filesystem::path proof_piggybox("./files/pig_bbox.off");
    CGAL::Surface_mesh<Point> sm;
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(source_pig, sm) || sm.is_empty()) {
        ASSERT_TRUE(false) << "Testfile not found.";
    }
    CGAL::Surface_mesh<Point> proof;
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(proof_piggybox, proof) || proof.is_empty()) {
        ASSERT_TRUE(false) << "Profile not found.";
    }

    pointcloud_xyzc points;
    for (const auto &v : vertices(sm)){
        points.push_back(std::make_tuple(sm.point(v), 0));
    }

    BBox proof_bbox;
    auto idx = 0;
    for(const auto& v : vertices(proof)) {
        auto tmp = proof.point(v);
        proof_bbox.get_vertices()[idx] = tmp;
        ++idx;
    }
    auto bbox = algo.fit_bounding_box(points);
    ASSERT_NEAR(bbox.volume(), proof_bbox.volume(), 0.01);
    }

// TODO still needed ?
//TEST (algorithm_testing , Algo_PCA) {
//    ASSERT_TRUE(false) << "Not implemented yet";
//}

//TODO This ist more a functional test
TEST (Python , Decompose3D_Pig_CorrectNumberOffSubBoxes) {
    mvbb::CGAL_MVBB<pointcloud_xyzc> mvbb;
    std::filesystem::path source_pig("./files/pig.off");
    CGAL::Surface_mesh<Point> sm;
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(source_pig, sm) || sm.is_empty()) {
        ASSERT_TRUE(false) << "Testfile not found.";
    }
    pointcloud_xyzc points;
    for (const auto &v : vertices(sm)) {
        points.push_back(std::make_tuple(sm.point(v), 0));
    }

    uint32_t levels = 3;

    TwoSplitStrategy two_split_strategy;
    auto tree = mvbb::decompose3D(points, &mvbb, mvbb::TargetSetting(levels), two_split_strategy);
    for(int k =0; k <= levels; ++k){
        uint32_t sk = 0;
        for(auto& node : tree.getNodes(k)) {
            std::filesystem::path cell_path_name( "./pig_lvl" + std::to_string(k) + "_" + std::to_string(sk) + ".off");
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
//TEST (algorithm_testing , Algo_MVBB ) {
//
//}
