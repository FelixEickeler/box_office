//
// Created by felix on 08.10.21.
//

#include "gtest/gtest.h"
#include "mvbb_algorithms.h"
#include "typedefs.h"
#include "data.h"
#include "rasterizer.h"

TwoSplitStrategy two_split;

TEST (Algorithm_TwoSplit , GetFirstAndLastSlot_Testpoints3x7AndOrientation_CorrectSlotSpans ) {
    auto [rasterizer, _] = GenerateRasterizer3x7<TwoSplitStrategy>();
    const auto test_points_x = (MatrixXu(2,6) <<  0, 0, 0, 0, 0, 2, 1,1,1,6,6,6).finished();
    auto res_x = rasterizer.Grid().get_first_and_last_slot<GridOrientation::X>();
    ASSERT_EQ(test_points_x, res_x);
    const auto test_points_y = (MatrixXu(2,6) <<  0, 0, 3, 0, 0, 3, 5,0,6,0,0,6).finished();
    auto res_y = rasterizer.Grid().get_first_and_last_slot<GridOrientation::Y>();
    ASSERT_EQ(test_points_y, res_y);
}

TEST (Algorithm_TwoSplit , CalculateBestSplits_TestPoints3x7_CorrectGRIDSplit ) {
    using namespace mvbb;
    auto [rasterizer, _]  = GenerateRasterizer3x7<TwoSplitStrategy>();
    auto [msr_ex, msr_ey] = two_split.calculate_best_splits(rasterizer.Grid());
    ASSERT_EQ(msr_ex.index[0], 2);
    ASSERT_EQ(msr_ex.area, 21);
    ASSERT_EQ(msr_ey.index[0], 0);
    ASSERT_EQ(msr_ey.area, 20);
}

/// Translate back to 3x7 grid;
TEST (Algorithm_TwoSplit , CalculateBestSplits_TestPoints3x7_CorrectSPACESplit ) {
    using namespace mvbb;
    auto [rasterizer, _]  = GenerateRasterizer3x7<TwoSplitStrategy>();
    auto [ms_ex, ms_ey] = rasterizer.best_splits();
    ProjectedLine tx{{3.5,0},{3.5,2}};
    // perfect cut in x right of 3 => 3 < 3.5 (which is next, cut)
    // perfect cut in y at above 0.0 => 0.0 < 0.33  (which is next, cut)

    ProjectedLine ty{{1, 1.0/3},{6.0,1.0/3}};
    ASSERT_FLOAT_EQ(ms_ex.front().cuts.front().start.x(), tx.start.x());
    ASSERT_FLOAT_EQ(ms_ex.front().cuts.front().start.y(), tx.start.y());
    ASSERT_FLOAT_EQ(ms_ex.front().cuts.front().end.x(), tx.end.x());
    ASSERT_FLOAT_EQ(ms_ex.front().cuts.front().end.y(), tx.end.y());
    ASSERT_FLOAT_EQ(ms_ex.front().area, 21.0 * 5.0/6*1.0/3);

    ASSERT_FLOAT_EQ(ms_ey.front().cuts.front().start.x(), ty.start.x());
    ASSERT_FLOAT_EQ(ms_ey.front().cuts.front().start.y(), ty.start.y());
    ASSERT_FLOAT_EQ(ms_ey.front().cuts.front().end.x(), ty.end.x());
    ASSERT_FLOAT_EQ(ms_ey.front().cuts.front().end.y(), ty.end.y());
    ASSERT_FLOAT_EQ(ms_ex.front().area, 21.0 * 5.0/6*1.0/3);
}

TEST (Algorithm_TwoSplit , CalculateBestSplits_TestPoints3x3_CorrectGRIDSplit) {
    auto [rasterizer, _] = GenerateRasterizer3x3<TwoSplitStrategy>();
    auto [msr_ex, msr_ey] = two_split.calculate_best_splits(rasterizer.Grid());
    ASSERT_EQ(msr_ex.index[0], 0);
    ASSERT_EQ(msr_ey.index[0], 0);
}