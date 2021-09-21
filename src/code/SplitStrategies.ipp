//
// Created by felix on 17.09.21.
//

#ifndef BOXOFFICE_SPLITSTRATEGIES_IPP
#define BOXOFFICE_SPLITSTRATEGIES_IPP

template<GridOrientation TDirection>
GridSplit TwoSplitStrategy::_calculate_best_split(const XY_Grid &grid) {
    auto slot_sizes = grid.get_first_and_last_slot<TDirection>();
    MatrixXu zero_stopper = (slot_sizes.row(1).array() == 0).template cast<uint32_t>() * UINT32_MAX;
    zero_stopper += slot_sizes.row(0);
    Eigen::Matrix<uint32_t, 2, 2> bbox_boundaries = Eigen::Matrix<uint32_t, 2, 2>::Zero();

    auto min_area = UINT32_MAX;
    uint32_t idx_min_area = 0;

    // cut between 0 and 1 of grid, last row does not after
    for (auto idx = 0; idx < slot_sizes.cols() - 1; ++idx) {
        uint32_t wbbox2 = grid.raster() - idx - 1;
        // check lower and upper bounds, basically grow e.g. push the boundaries up / down
        if (bbox_boundaries(0, 0) > slot_sizes(0, idx)) bbox_boundaries(0, 0) = slot_sizes(0, idx);
        if (bbox_boundaries(1, 0) < slot_sizes(1, idx)) bbox_boundaries(1, 0) = slot_sizes(1, idx);

        // second bounding box => only evaluate non zero:zero columns, for minimum => zero_stopper!
        bbox_boundaries(0, 1) = zero_stopper.topRightCorner(1, wbbox2).minCoeff();
        bbox_boundaries(1, 1) = slot_sizes.bottomRightCorner(1, wbbox2).maxCoeff();

        // size of first bounding box + size of second bounding box: width * max(span) + width2 * max(span2)
        auto area = (idx + 1) * (bbox_boundaries(1, 0) - bbox_boundaries(0, 0)) +
                    wbbox2 * (bbox_boundaries(1, 1) - bbox_boundaries(0, 1));

        if (area < min_area) {
            min_area = area;
            idx_min_area = idx;
        }
    }
    return GridSplit{
        grid.index2line<TDirection>(idx_min_area),
        min_area,
        idx_min_area,
        TDirection};
}


#endif //BOXOFFICE_SPLITSTRATEGIES_H
