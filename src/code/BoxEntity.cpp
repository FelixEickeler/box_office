//
// Created by felix on 19.07.21.
//

#include "BoxEntity.h"

std::vector<mvbb::FitAndSplitNode<pointcloud_xyzc>> BoxEntity::decompose(int depth, float gain_threshold) {
    if(decomposition_level < depth || used_gain != gain_threshold) {
        mvbb::Algo_MVBB<pointcloud_xyzc> cgal_min_volumne;
        this->tree_hierarchy = mvbb::decompose3D(this->points, &cgal_min_volumne, depth, gain_threshold);
        decomposition_level = depth;
        used_gain = gain_threshold;
        return this->tree_hierarchy.get_finalized();
    }
    else{
//        std::vector<mvbb::FitAndSplitNode<pointcloud_xyzc>> collection;
        auto previous_levels = this->tree_hierarchy.get_finalized(depth-1);
        auto this_level = this->tree_hierarchy.getNodes(depth);
        for(auto& tl : this_level){
            previous_levels.push_back(tl);
        }
        return previous_levels;
    }
}

uint32_t BoxEntity::get_id() const {
    return this->id;
}

pointcloud_xyzc BoxEntity::get_points() const {
    return points;
}
