//
// Created by felix on 17.09.21.
//

#ifndef BOXOFFICE_SPLITSTRATEGIES_IPP
#define BOXOFFICE_SPLITSTRATEGIES_IPP

template<GridOrientation TDirection>
GridSplit TwoSplitStrategy::_calculate_best_split(const XY_Grid &grid) {
    // slots means cells !
    auto slot_sizes = grid.get_first_and_last_slot<TDirection>();
    MatrixXu zero_stopper = (slot_sizes.row(1).array() == 0).template cast<uint32_t>() * UINT32_MAX;
    // this overwrites all zeroes  so the minimum will not appear to be zero if a column is empty
    zero_stopper += slot_sizes.row(0);
    Eigen::Matrix<uint32_t, 2, 2> bbox_boundaries = Eigen::Matrix<uint32_t, 2, 2>::Zero();
    int _min_area = INT_MAX;
    int idx_min_area = 0;

    // cut the grid at appropriate coordinates.
    // First possible cut stats before the last element idx -1
    // Last possible cut ends before the first element ...cols() - 1 == 0

    // Walk over all columns, idx is the cell entry, area is calculated on 0 -> cell. So idx is including the current cell
    // This means BEFORE Minimum and AFTER maximumn
    for (auto icol = 0; icol < slot_sizes.cols() - 1; ++icol) {
        uint32_t width_bbox2 = (grid.raster()- 1) - icol; // because the last cell is raster - 1
        // check lower and upper bounds, basically grow e.g. push the boundaries up / down
        if (bbox_boundaries(0, 0) > slot_sizes(0, icol)) bbox_boundaries(0, 0) = slot_sizes(0, icol);
        if (bbox_boundaries(1, 0) < slot_sizes(1, icol)) bbox_boundaries(1, 0) = slot_sizes(1, icol);

        // second bounding box => only evaluate non zero:zero columns, for minimum => zero_stopper!
        bbox_boundaries(0, 1) = zero_stopper.topRightCorner(1, width_bbox2).minCoeff();
        bbox_boundaries(1, 1) = slot_sizes.bottomRightCorner(1, width_bbox2).maxCoeff();

        // size of first bounding box + size of second bounding box: width * max(span) + width2 * max(span2)
        auto area = (icol + 1) * (bbox_boundaries(1, 0) - bbox_boundaries(0, 0)) +
                    width_bbox2 * (bbox_boundaries(1, 1) - bbox_boundaries(0, 1));

        if (area < _min_area) {
            _min_area = area;
            idx_min_area = icol;
        }
    }
    return GridSplit{
        {grid.index2line<TDirection>(idx_min_area)},
        _min_area,
        {idx_min_area},
        TDirection};
}

template <class TMatrix>
inline uint calculate_box_height(TMatrix& box) {
    uint max = box.row(1).maxCoeff();
    uint min = box.row(0).minCoeff();
    return max > min ? max - min : 0;
}
/// This function enables a triple split scenario to enable concave hulls (kind of).
/// Main Idea: Two pointers named slow and fast (e.g. outer and inner loop)
/// Slow walks 0 to 0.5
/// Fast walks from -1 to slow
/// If they meet 0.5 all splits are calculated as both "splits" are interchangable.
/// The box area is calulated by the sum of the three boxes
///
/// \tparam TDirection
/// \param grid
/// \return
template<GridOrientation TDirection>
GridSplit TripleSplitStrategy::_calculate_best_split(const XY_Grid &grid) {
    // slots means cells !
    auto slot_sizes = grid.get_first_and_last_slot<TDirection>();
    slot_sizes.row(0) = slot_sizes.row(0) + ((slot_sizes.row(1).array() == 0).template cast<unsigned int>() * UINT_MAX).matrix();
    // this overwrites all zeroes  so the minimum will not appear to be zero if a column is empty
    Eigen::Matrix<uint32_t, 2, 2> bbox_boundaries = Eigen::Matrix<uint32_t, 2, 2>::Zero();
    int _min_area = INT_MAX;
    std::vector<int> idx_min_area = {0, int(slot_sizes.cols())};

    for(auto slow = 1; slow < static_cast<int>(slot_sizes.cols())-1; ++slow){
        for(auto fast = static_cast<int>(slot_sizes.cols()-1); fast > slow; --fast){
            // define 3 boxes
            // row, column => 2 , 6
            auto box1 = slot_sizes.block(0,0, 2, slow);
            auto box2 = slot_sizes.block(0, slow, 2, fast-slow);
            auto box3 = slot_sizes.block(0,fast, 2, slot_sizes.cols()-fast);

            // calculate the are by sum
            auto area = calculate_box_height(box1) * slow
                    + calculate_box_height(box2) * (fast-slow)
                    + calculate_box_height(box3) * (slot_sizes.cols()-fast);

            if (area < _min_area || (area == _min_area && (fast-slow) < (idx_min_area[1] - idx_min_area[0]))) {
                _min_area = area;
                idx_min_area = {slow, fast};
            }
        }
    }

    // TODO fix grid_line
    return GridSplit{
            {grid.index2line<TDirection>(idx_min_area[0]), grid.index2line<TDirection>(idx_min_area[1])},
            _min_area,
            idx_min_area,
            TDirection};
}


#endif //BOXOFFICE_SPLITSTRATEGIES_H
