//
// Created by felix on 17.09.2021.
//
#include "SplitStrategies.h"
using namespace mvbb;

GridSplits TwoSplitStrategy::calculate_best_splits(const XY_Grid &grid) {
    GridSplits splits{
        {_calculate_best_split<GridOrientation::X>(grid)},
        {_calculate_best_split<GridOrientation::Y>(grid)}};
    return splits;
}

