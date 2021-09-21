//
// Created by felix on 17.09.21.
//

#ifndef BOXOFFICE_SPLITSTRATEGIES_H
#define BOXOFFICE_SPLITSTRATEGIES_H

#include "xy_grid.h"
#include "typedefs.h"

using namespace mvbb;

    class SplitStrategy {
        public:
            virtual GridSplits calculate_best_splits(const XY_Grid &grid) = 0;
    };

    class TwoSplitStrategy : public SplitStrategy {
        private:
            template<GridOrientation TDirection>
            GridSplit _calculate_best_split(const XY_Grid &grid);
        public:
            GridSplits calculate_best_splits(const XY_Grid &grid) override;
    };

    #include "SplitStrategies.ipp"

#endif //BOXOFFICE_SPLITSTRATEGIES_H
