//
// Created by felix on 19.09.2021.
//
#ifndef BOXOFFICE_XY_GRID_H
#define BOXOFFICE_XY_GRID_H
#include "rasterizer_definitions.h"
#include "typedefs.h"

namespace mvbb {

    struct XY_Grid {
        MatrixXu data;

        explicit XY_Grid(long raster) : data(MatrixXu::Zero(raster, raster)) {}

        [[nodiscard]] size_t raster() const;

        /// This method returns a vector of spans representing the occupied space of the grid.
        /// It will be either provide these as column : GridOrientation::X  or row : GridOrientation::Y
        /// \tparam TDirection :  true for row; false for column
        /// \return Eigen::Vector2i<TRaster : v(n,0) start of the span and v(n,1) end of it
        template<GridOrientation TDirection>
        auto get_first_and_last_slot() const;

        /// This method will convert a index (of the cut in the ruster) to a 2D Line in the grid space.
        /// \tparam TDirection
        /// \param index
        /// \return GridLine : with start and end defined
        template<GridOrientation TDirection>
        auto index2line(uint32_t index) const;
    };

    #include "xy_grid.ipp"
}

#endif //BOXOFFICE_XY_GRID_H
