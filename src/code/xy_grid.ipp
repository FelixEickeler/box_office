//
// Created by felix on 19.09.2021.
//

#ifndef BOXOFFICE_XY_GRID_IPP
#define BOXOFFICE_XY_GRID_IPP

using namespace mvbb;

// It is not transformed in the a real coordinate system (e.g. cutting plane)
template<GridOrientation TDirection>
auto mvbb::XY_Grid::get_first_and_last_slot() const{
    MatrixXu _span = MatrixXu::Zero(2, data.outerSize());

    for (auto i = 0; i < this->raster(); ++i) {
        bool first_set = false;
        for (auto j = 0; j < data.outerSize(); ++j) {
            if constexpr(TDirection == GridOrientation::Y){
                if (data(i, j) != 0) {
                    if (!first_set) {
                        _span(0, i) = j;
                        first_set = true;
                    }
                    _span(1, i) = j + 1;
                }
            } else {
                if (data(j, i) != 0) {
                    if (!first_set) {
                        _span(0, i) = j;
                        first_set = true;
                    }
                    _span(1, i) = j + 1;
                }
            }
        }
    }
    return _span;
}

template<GridOrientation TDirection>
auto mvbb::XY_Grid::index2line(uint32_t index) const{
    GridLine line;
    if constexpr (TDirection == GridOrientation::Y) {
        line.start << 0, index;
        line.end << raster() - 1, index;
    } else {
        line.start << index, 0;
        line.end << index, raster() - 1;
    }
    return line;
}

#endif //BOXOFFICE_XY_GRID_IPP
