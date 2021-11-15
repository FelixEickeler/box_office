//
// Created by felix on 19.07.21.
//

#ifndef BOXOFFICE_BOXENTITY_H
#define BOXOFFICE_BOXENTITY_H
#include <cstdint>
#include <utility>
#include "typedefs.h"
#include "mvbb_algorithms.h"
#include "FitAndSplitHierarchy.h"
#include "TargetSetting.h"

using namespace boxy;

class BoxEntity {
    uint32_t id;
    pointcloud_xyzc points;
    FitAndSplitHierarchy<pointcloud_xyzc> tree_hierarchy;
    std::string name;
    TargetSetting last_setting = {-1, 0.0};

    public:
        explicit BoxEntity(uint32_t id, std::string name, pointcloud_xyzc points) : id(id), name(std::move(name)), points(std::move(points)){}

        //TODO sanity checks
        std::vector<FitAndSplitNode<pointcloud_xyzc>> decompose(TargetSetting target_setting, SplitStrategy& split_strategy);

        [[nodiscard]] uint32_t get_id() const;

        [[nodiscard]] std::string get_name() const;

        [[nodiscard]] pointcloud_xyzc get_points() const;
};


#endif //BOXOFFICE_BOXENTITY_H
