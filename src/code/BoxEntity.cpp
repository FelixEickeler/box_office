//
// Created by felix on 19.07.21.
//

#include "BoxEntity.h"
#include "TargetSetting.h"
#include <spdlog/spdlog.h>

std::vector<FitAndSplitNode<pointcloud_xyzc>> BoxEntity::decompose(TargetSetting target_setting, SplitStrategy& split_strategy) {
    if(!last_setting.only_inferior_gain(target_setting)) {
        spdlog::debug("Hierarchy is not valid: Recalculate the bounding box hierarchy");
        mvbb::CGAL_MVBB<pointcloud_xyzc> cgal_min_volume;
        auto target = TargetSetting(target_setting.kappa , target_setting.gain_threshold);
        this->tree_hierarchy = mvbb::decompose3D(this->points, &cgal_min_volume, target, split_strategy);
        last_setting = target_setting;
        return this->tree_hierarchy.get_finalized();
    }
    else{
        spdlog::debug("Hierarchy is valid: reusing the existing bounding box hierarchy ");
        auto previous_levels = this->tree_hierarchy.get_finalized(target_setting.kappa );
        auto this_level = this->tree_hierarchy.getNodes(target_setting.kappa );
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

std::string BoxEntity::get_name() const {
    return name;
}
