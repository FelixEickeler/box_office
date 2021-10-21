//
// Created by felix on 09.07.2021.
//

#include "gtest/gtest.h"
#include "mvbb_algorithms.h"
#include "typedefs.h"
#include "data.h"
#include "rasterizer.h"
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

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

TEST (Discretization , GetFirstAndLastSlot_Testpoints3x3AndOrientation_CorrectSlotSpans ) {
    auto [rasterizer, _] = GenerateRasterizer3x3<TwoSplitStrategy>();
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