//
// Created by felix on 18.07.2021.
//
#define PRODUCTION 1
#include <spdlog/spdlog.h>

#include <utility>
#include "spdlog/stopwatch.h"
#include "mvbb_algorithms.h"
#include "SplitStrategies.h"
#include "TargetSetting.h"
#include "result_writer.h"

using namespace mvbb;


FitAndSplitHierarchy<TPointCloudType> mvbb::decompose3D(TPointCloudType &points3D, Algo_Base<TPointCloudType> *const bbox_algorithm, TargetSetting const target_settings, SplitStrategy& split_strategy) {
    spdlog::trace("Processing: {} Points, with {} kappa", points3D.size(),  target_settings.kappa);
    spdlog::set_pattern("\t[%^%l%$][%S,%es] %v");
    spdlog::stopwatch sw;

    auto result_writer = make_result_writer(target_settings.base_path);
    auto initial_bbox = bbox_algorithm->fit_bounding_box(points3D);
    FitAndSplitHierarchy<TPointCloudType> tree;
    tree.emplace_back(0, initial_bbox, points3D.begin(), points3D.end());
    Path result_postfix;

    for(auto depth=0; depth< target_settings.kappa; ++depth){
        int node_counter = 0;
        if(tree.depth() <= depth) break;
        auto current_hierarchy_volume = tree.current_hierarchy_volume();
        spdlog::debug("Decomposing level: {0} with {1} max gain and a hierarchy volume {2}", depth, target_settings.gain_threshold, current_hierarchy_volume / initial_bbox.volume());

        for(auto& fas_node : tree.getNodes(depth)) {   //std::for_each(all_parents.begin(), all_parents.end(), [&tree, bbox_algorithm, depth, &node_counter](auto& fas_node){
            #ifdef PRODUCTION
            if(target_settings.output_boxes) {
                result_postfix = fmt::format("depth={}-node={}", std::to_string(depth), node_counter);
                result_writer->set_path_addendum(result_postfix);
                result_writer->write(ResultType::BoundingBox(), fas_node.bounding_box, fas_node.final);
            }
            #endif

            auto bbox = fas_node.bounding_box;
            BoxSplits best_bbox_splits;
            // A=0; B=1; C=2 => see typdef::BoxFaces;
            // calculate each split for evey direction on evey face 3 faces x 2
            for (auto face = 0; face < 3; ++face) {
                // project, change coordinate system and ...
                auto current_face = static_cast<boxy::BoxFaces>(face);
                auto projection_plane = bbox.get_plane(current_face);
                auto coordinate_system2D = bbox.get_plane_coordinates2D(current_face);
                // ... rasterize !

                auto raster = Discretization::create_discretization(split_strategy, bbox, coordinate_system2D, 512);
                auto bad_counter = 0;

                std::vector<Point2D> projected_2D;
                for(auto& point : fas_node.points){
                    auto proj = projection_plane.projection(std::get<0>(point)); //project to plane (3D)
                    auto point2d = coordinate_system2D.project_onto_plane(proj); //project to plane (2D) & new coordinate system
                    raster.insert(point2d);
                }

               #ifdef PRODUCTION
                if(target_settings.output_grids){
                    result_postfix =fmt::format("node={}-face={}", std::to_string(node_counter), std::to_string(face));
                    result_writer->set_path_addendum(result_postfix);
                    result_writer->write(ResultType::Grid(), raster, coordinate_system2D);
                }
               #endif

                auto cell_depth = bbox.get_depth(current_face);
                best_bbox_splits[current_face] = raster.best_splits();
                //Output possible plane3s
                #if false
                for(auto i = 0; i < 2; ++i) {
                        auto cut = coordinate_system2D.project_to_global(all_splits[face * 2 + i].end_cut);
                        auto fulcrum = coordinate_system2D.project_to_global(all_splits[face * 2 + i].begin_cut);
                        auto origin = coordinate_system2D.project_to_global(grid.from_grid(Point2D(0,0)));
                        auto[plane_mesh, plane_system] = Plane2Mesh(fulcrum, origin, cut, i > 0);

                        std::filesystem::path cplan_off("./cutting_plane_" + std::to_string(depth) + "_" + std::to_string(face) + "_" + std::to_string(i) + ".off");
                        if (!CGAL::IO::write_OFF(cplan_off, plane_mesh, CGAL::parameters::stream_precision(6))) {
                            std::cout << "oh no2\n";
                        }
//                        std::filesystem::path plane_system_path("./cutting_plane_system" + std::to_string(depth) + "_" + std::to_string(face) + "_" + std::to_string(i) + ".off");
//                        if (!CGAL::IO::write_OFF(plane_system_path, plane_system, CGAL::parameters::stream_precision(6))) {
//                            std::cout << "oh no3\n";
//                        }
//                        GTEST_COUT << "CUT: " << depth << "Plane: " << std::to_string(face) << "direction:" << i << " Volumne" << std::to_string(all_splits[face*2+i].area)
//                               << std::endl;
//                }
                    }
                #endif
            }
            // select_best_split & get the face its on
            auto [best_split, selected_face] = best_bbox_splits.superior_split();


            // reinstate the PlaneCoordinate System
            auto coordinate_system2D = bbox.get_plane_coordinates2D(selected_face);
            // planes ordered from 0 -> raster
            auto cutting_planes = best_split.project_cuts_and_create_planes(coordinate_system2D);

            #ifdef PRODUCTION
            if(target_settings.output_cuts) {
                result_postfix = fmt::format("depth={}-node={}-face={}", std::to_string(depth),
                                             node_counter, static_cast<int>(selected_face));
                result_writer->set_path_addendum(result_postfix);
                result_writer->write(ResultType::Plane(), best_split, coordinate_system2D);
            }
            #endif

            // Potential bug in triple split
            // function that rearranges the underling points accordingly to proposal
            // TODO make function from lambda
            auto calc_begin_of_next_point_partition = [fas_node](const Plane_3& next_cutting_plane, auto points_begin) {
                return std::partition(points_begin, fas_node.points.end(),
                           [&next_cutting_plane](auto &point) {
                               return !BBox::has_on_or_negative_side(next_cutting_plane, std::get<0>(point));
                           });
            };

            // scope defintions
            std::vector<typename TPointCloudType::iterator> bucket_boundaries = {fas_node.points.begin()};
            std::vector<int> bucket_sizes;

            std::vector<BBox> proposed_boxes;
            float proposed_volume = 0;
            float gain = -1;

            // Reject if there are little points ...
            bool low_point_number = false;
            for(const auto& next_cutting_plane : cutting_planes){
                bucket_boundaries.push_back(calc_begin_of_next_point_partition(next_cutting_plane, bucket_boundaries[bucket_boundaries.size()-1]));
                auto points_in_this_bucket = bucket_sizes.emplace_back(std::distance(bucket_boundaries[bucket_boundaries.size()-2], bucket_boundaries.back()));
                if(points_in_this_bucket > target_settings.minimum_point_per_box){
                    low_point_number = true;
                }
            }
            bucket_boundaries.push_back(fas_node.points.end());
            bucket_sizes.emplace_back(std::distance(bucket_boundaries[bucket_boundaries.size()-2], bucket_boundaries.back()));

            auto bucket_stats_string = std::accumulate(std::next(bucket_sizes.begin()), bucket_sizes.end(), std::to_string(bucket_sizes[0]),
                [](const std::string& a, int size){
                    return a + ", " + std::to_string(size);
                });

            //... of if the volume is too low ...
            for(auto right_bucket_boundary_idx=1; right_bucket_boundary_idx < bucket_boundaries.size(); ++right_bucket_boundary_idx) {
                auto left_bucket_boundary = bucket_boundaries[right_bucket_boundary_idx-1];
                auto right_bucket_boundary = bucket_boundaries[right_bucket_boundary_idx];

                TPointCloudType this_bucket(left_bucket_boundary, right_bucket_boundary);
                auto& proposed_bbox = proposed_boxes.emplace_back(bbox_algorithm->fit_bounding_box(this_bucket));
                // check if the minimum volume is adhered otherwise finalize
                if(bbox.volume() < initial_bbox.volume() / float(target_settings.minimal_initial_volume_divider)){
                    goto skip_rest_finalize_and_goto_next_node;
                }
                //spdlog::debug("Current_Volumne:{} => Points: {}", proposed_bbox.volume(), this_bucket.size());
                proposed_volume += proposed_bbox.volume();
            }

            //... or of the gain is too low ...
            gain = (current_hierarchy_volume - fas_node.bounding_box.volume() + proposed_volume) / current_hierarchy_volume;
            if(gain < target_settings.gain_threshold){
                int dive = depth+1;
                for(auto pbox_idx=0; pbox_idx < proposed_boxes.size(); ++pbox_idx){
                    auto left_bucket_boundary = bucket_boundaries[pbox_idx];
                    auto right_bucket_boundary = bucket_boundaries[pbox_idx+1];
                    tree.emplace_back(dive, proposed_boxes[pbox_idx], left_bucket_boundary, right_bucket_boundary);
                }
            }

            // node was finalized before e.g. protector for dead end !
            if(fas_node.final){
                skip_rest_finalize_and_goto_next_node:;
                fas_node.final = true;
            }


            spdlog::trace("Box: {0}\t Points: {1}\t=>{2}\t| gain: {3}\t | status: {4}", node_counter, fas_node.points.size(), bucket_stats_string, gain, fas_node.final ? "rejected" : "accepted");
            node_counter++;
        }
    }
    auto& final_hierachy_nodes = tree.max_depth();
    // finalize last level of nodes
    for(auto& node : tree.max_depth()){
        node.final = true;
    }


    spdlog::debug("Decomposition took: {} seconds. Final hierarchy volume:\t {}", sw, tree.current_hierarchy_volume() / initial_bbox.volume());
    spdlog::set_pattern("%+");
    return tree;
}


