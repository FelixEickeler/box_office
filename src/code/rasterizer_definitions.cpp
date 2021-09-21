//
// Created by felix on 19.09.2021.
//

#include "rasterizer_definitions.h"

std::array<float, 4> mvbb::minmax2D(const BBox &bbox, const CoordinateSystem2D &coordinate_system2D) {
    std::array<float, 4> min_max2D{1.0e10, -1.0e-10,1.0e-10,-1.0e-10};
    for(auto& v : bbox.get_vertices()){
        auto p2d = coordinate_system2D.project_onto_plane(v);
        auto x = p2d.x(); auto y = p2d.y();
        if(x < min_max2D[0]){
            min_max2D[0] = x;
        }
        else if(x > min_max2D[2]){
            min_max2D[2] = x;
        }
        if(y < min_max2D[1]){
            min_max2D[1] = y;
        }
        else if(y > min_max2D[3]){
            min_max2D[3] = y;
        }
    }
    return min_max2D;
}

mvbb::BoxSplits::Return_Best_Split mvbb::BoxSplits::superior_split() {
    // Which is the best split
    auto bs_iter = std::min_element(_box_splits.begin()->x.begin(), _box_splits.end()->y.end(), min_area);
    BoxFaces which_face;

    // Which face ?
    for (int i = 3; i > 0; --i) {
        if(std::distance(bs_iter, _box_splits[i].x.begin()) >= 0){
            which_face = static_cast<BoxFaces>(i);
            break;
        }
    }
    return {*bs_iter, which_face};
}

mvbb::ProjectedSplit mvbb::ProjectedSplits::best_x() const{
    return *std::min_element(x.begin(), x.end(), min_area);
}

mvbb::ProjectedSplit mvbb::ProjectedSplits::best_y() const {
    return *std::min_element(y.begin(), y.end(), min_area);
}

mvbb::ProjectedSplit mvbb::ProjectedSplits::best_xy() const{
    return *std::min_element(x.begin(), y.end(), min_area);
}
