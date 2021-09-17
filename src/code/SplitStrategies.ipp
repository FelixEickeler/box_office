//
// Created by felix on 17.09.21.
//

#ifndef BOXOFFICE_SPLITSTRATEGIES_IPP
#define BOXOFFICE_SPLITSTRATEGIES_IPP

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
        uint32_t wbbox2 = this->raster() - idx - 1;
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

#endif //BOXOFFICE_SPLITSTRATEGIES_H
