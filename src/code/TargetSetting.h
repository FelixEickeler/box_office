//
// Created by felix on 20.10.21.
//

#ifndef BOXOFFICE_TARGETSETTING_H
#define BOXOFFICE_TARGETSETTING_H

struct TargetSetting {
    const int kappa;
    const float gain_threshold;
    const int minimum_point_per_box;
    const int minimal_initial_volume_divider;
    std::filesystem::path base_path;
    bool subdivide = false;
    bool output_planes = false;
    bool output_grids = false;
    bool output_cuts = false;
    bool output_boxes = false;

    TargetSetting(int kappa_, float gain_threshold_=0.99, int minimum_point_per_box_ = 10,
                  int minimal_initial_volume_divider_ = 1000, std::string output_cutting_plane_path_="");
    TargetSetting& operator= (const TargetSetting& f);

    bool all_other_equal(TargetSetting& that) const;
    bool only_inferior_gain(TargetSetting& that) const;
    bool operator==(TargetSetting& that) const;
    bool operator!=(TargetSetting& that) const;
};

#include <functional>
#include <algorithm>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/bounding_box.h>
#include <iterator>
#include <fstream>
#include <CGAL/IO/write_off_points.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Origin.h>
#include <filesystem>
#include "typedefs.h"
#include "helpers.h"
#include "rasterizer.h"
#include "FitAndSplitHierarchy.h"
#include "SplitStrategies.h"
#include "rasterizer_definitions.h"

#endif //BOXOFFICE_TARGETSETTING_H
