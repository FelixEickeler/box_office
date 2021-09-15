//
// Created by felix on 14.09.2021.
//

template<bool TDirection>
auto mvbb::Grid::best_split() {
    auto slot_sizes = get_first_and_last_slot<TDirection>();
    MatrixXu zero_stopper = (slot_sizes.row(1).array() == 0).template cast<uint32_t>() * UINT32_MAX;
    zero_stopper += slot_sizes.row(0);
    Eigen::Matrix<uint32_t, 2, 2> bbox_boundaries = Eigen::Matrix<uint32_t, 2, 2>::Zero();

    auto min_area = UINT32_MAX;
    uint32_t idx_min_area = 0;

    // cut between 0 and 1 of grid, last row does not after
    for (auto idx = 0; idx < slot_sizes.cols() - 1; ++idx) {
        uint32_t wbbox2 = this->parent._raster - idx - 1;
        if (bbox_boundaries(0, 0) > slot_sizes(0, idx)) bbox_boundaries(0, 0) = slot_sizes(0, idx);
        if (bbox_boundaries(1, 0) < slot_sizes(1, idx)) bbox_boundaries(1, 0) = slot_sizes(1, idx);

        // only evaluate non zero:zero columns, for minimum => zero_stopper!
        bbox_boundaries(0, 1) = zero_stopper.topRightCorner(1, wbbox2).minCoeff();
        bbox_boundaries(1, 1) = slot_sizes.bottomRightCorner(1, wbbox2).maxCoeff();

        auto area = (idx + 1) * (bbox_boundaries(1, 0) - bbox_boundaries(0, 0)) +
                    wbbox2 * (bbox_boundaries(1, 1) - bbox_boundaries(0, 1));

        if (area < min_area) {
            min_area = area;
            idx_min_area = idx;
        }
    }
    return BestGridSplit{min_area, idx_min_area};
}

template<bool TDirection>
BestSplit mvbb::Space::best_split(float cell_depth) {
    auto bgs = parent.get().grid.best_split<TDirection>();
    float cell_area = parent.grid.step_x() * parent.grid.step_y();
    Vector2f line{0, parent._raster - 1};

    Eigen::Matrix<uint32_t, 2, 1> begin_cut_grid;
    Eigen::Matrix<uint32_t, 2, 1> end_cut_grid;
    if constexpr (TDirection) { // true being direction_ey
        begin_cut_grid << 0, bgs.index;
        end_cut_grid << parent._raster - 1, bgs.index;
    } else {
        begin_cut_grid << bgs.index, 0;
        end_cut_grid << bgs.index, parent._raster - 1;
    }
    auto begin_cut = from_grid(begin_cut_grid);
    auto end_cut = from_grid(end_cut_grid);
    // TODO another quickfix
    auto origin = from_grid(Point2D(0, 0));
    return BestSplit{bgs.area * cell_area, begin_cut, end_cut, origin};
}

template<bool TDirection>
auto mvbb::Grid::get_first_and_last_slot() {
    MatrixXu _span = MatrixXu::Zero(2, this->parent._raster);

    for (auto i = 0; i < this->parent._raster; ++i) {
        bool first_set = false;
        for (auto j = 0; j < this->parent._raster; ++j) {
            if constexpr(TDirection) {
                if (this->parent._data(i, j) != 0) {
                    if (!first_set) {
                        _span(0, i) = j;
                        first_set = true;
                    }
                    _span(1, i) = j + 1;
                }
            } else {
                if (this->parent._data(j, i) != 0) {
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
