//
// Created by felix on 14.09.2021.
//



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
    MatrixXu _span = MatrixXu::Zero(2, data.outerSize());

    for (auto i = 0; i < this->raster(); ++i) {
        bool first_set = false;
        for (auto j = 0; j < data.outerSize(); ++j) {
            if constexpr(TDirection) {
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
