//
// Created by felix on 08.10.21.
//

#include <BoxScene.h>
#include "gtest/gtest.h"
#include "mvbb_algorithms.h"
#include "typedefs.h"
#include "data.h"
#include "rasterizer.h"
#include "BoxScene.h"

// data
/*
 * blocks move => results should always be the correct position of the block
 */

TEST (Algorithm_TripleSplit , CalculateBestSplits_MovingBlocks_CorrectSplits) {
    using namespace mvbb;

    XY_Grid grid(9);
    MatrixXu block(3,3);

    block << 1,1,1,
             1,1,1,
             1,1,1;

    for(short perm; perm <=9; ++perm){
        grid.data.setZero();
        std::array<Eigen::Vector2i,3> block_positions;
        for(auto bin : {0,1,2}){
            auto placement = perm & (1 << bin);
            grid.data.block(placement, bin*3,3,3) = block;
            block_positions[bin] << bin*3, placement;
        };
        TripleSplitStrategy split_strategy;
        auto [bsx, bsy] = split_strategy.calculate_best_splits(grid);
        ASSERT_EQ(bsx.area, 27);
    }
}

// 1, 0, 0, 0, 0, 0,        +3 ==>19
// 1, 0, 0, 0, 0, 0,
// 1, 0, 0, 0, 0, 0, __x1:3
// 1, 0, 1, 0, 0, 1,        +12
// 1, 0, 1, 0, 0, 1, __x2:5
// 0, 0, 1, 0, 0, 1         +4
//  |y1|y2  :1,:2
//+5 +0 +12   => 17

TEST (Algorithm_TripleSplit , CalculateBestSplits_TestPoints3x7_CorrectGRIDSplit ) {
    using namespace mvbb;
    TripleSplitStrategy triple;
    auto [rasterizer, _]  = GenerateRasterizer3x7<TripleSplitStrategy>();
    auto [msr_ex, msr_ey] = triple.calculate_best_splits(rasterizer.Grid());
    ASSERT_EQ(msr_ex.index[0], 3);
    ASSERT_EQ(msr_ex.index[1], 5);
    ASSERT_EQ(msr_ex.area, 19);
    ASSERT_EQ(msr_ey.index[0], 1);
    ASSERT_EQ(msr_ey.index[1], 2);
    ASSERT_EQ(msr_ey.area, 17);
}

TEST (Algorithm_TripleSplit , Decompose3D_Bunny_NoThrow) {
    auto eval = create_scene("./files/bunny_classy_head.txt",
                             "./files/bunny_classy_head.ol");
    auto entity = eval.get_object(0);
    mvbb::CGAL_MVBB<boxy::pointcloud_xyzc> algo;
    auto epoints = entity.get_points();
    TripleSplitStrategy split_strategy;
    auto target_setting = TargetSetting(16, 0.995,10,1000,"/home/boxy/dev/debug/bunny_tripple_test");
    spdlog::set_level(spdlog::level::trace);
    target_setting.output_grids=true;
    target_setting.output_cuts=true;
    target_setting.output_boxes=true;

    mvbb::decompose3D(epoints, &algo, target_setting, split_strategy);
    ASSERT_NO_THROW();
}

TEST (Algorithm_TripleSplit , Decompose3D_424_NoThrow) {
    auto eval = create_scene("./files/424.txt",
                             "./files/boxes.ol");
    auto entity = eval.get_object(0);
    mvbb::CGAL_MVBB<boxy::pointcloud_xyzc> algo;
    auto epoints = entity.get_points();
    TripleSplitStrategy split_strategy;
    auto target_setting = TargetSetting(2, 0.99,20,1000,"/home/boxy/dev/debug/404");
    spdlog::set_level(spdlog::level::trace);
    target_setting.output_grids=true;
    target_setting.output_cuts=true;
    target_setting.output_boxes=true;
    mvbb::decompose3D(epoints, &algo, target_setting, split_strategy);
    ASSERT_NO_THROW();
}