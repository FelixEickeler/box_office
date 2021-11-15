//
// Created by felix on 19.09.2021.
//

#include "xy_grid.h"

size_t mvbb::XY_Grid::raster() const {
    return data.innerSize();
}