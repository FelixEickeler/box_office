//
// Created by felix on 20.10.21.
//

#include "SplitStrategies.h"
#include "mvbb_algorithms.h"
#include "spdlog/stopwatch.h"
#include <utility>
#include <spdlog/spdlog.h>
#include "TargetSetting.h"

TargetSetting::TargetSetting(int kappa_, float gain_threshold_, int minimum_point_per_box_, int minimal_initial_volume_divider_, std::string output_path_) :
    kappa(kappa_), gain_threshold(gain_threshold_), minimum_point_per_box(minimum_point_per_box_), minimal_initial_volume_divider(minimal_initial_volume_divider_), base_path(std::move(output_path_)){}

bool TargetSetting::all_other_equal(TargetSetting& that) const{
    return this->minimal_initial_volume_divider == that.minimal_initial_volume_divider &&
           this->minimum_point_per_box == that.minimum_point_per_box &&
           this->base_path == that.base_path;
}

bool TargetSetting::operator==(TargetSetting& that) const {
    return this->kappa == that.kappa && this->gain_threshold == that.gain_threshold && all_other_equal(that);
}

bool TargetSetting::operator!=(TargetSetting& that) const {
    return !(*this == that);
}

bool TargetSetting::only_inferior_gain(TargetSetting &that) const {
    return that.kappa <= kappa && all_other_equal(that);
}

TargetSetting &TargetSetting::operator=(const TargetSetting &f) {
    return *this;
}