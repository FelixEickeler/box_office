//
// Created by felix on 18.07.2021.
//
#include <spdlog/spdlog.h>
#include "spdlog/stopwatch.h"
#include "mvbb_algorithms.h"
#include "SplitStrategies.h"

using namespace mvbb;


FitAndSplitHierarchy<TPointCloudType> mvbb::decompose3D(TPointCloudType &points3D, Algo_Base<TPointCloudType> *const bbox_algorithm, Target_Setting const target_settings) {
    spdlog::trace("Processing: {} Points, with {} kappa", points3D.size(),  target_settings.kappa);
    spdlog::set_pattern("\t[%^%l%$][%S,%es] %v");
    spdlog::stopwatch sw;

    auto initial_bbox = bbox_algorithm->fit_bounding_box(points3D);
    FitAndSplitHierarchy<TPointCloudType> tree;
    tree.emplace_back(0, initial_bbox, points3D.begin(), points3D.end());

    for(auto depth=0; depth< target_settings.kappa; ++depth){
        int node_counter = 0;
        if(tree.depth() <= depth) break;
        auto current_hierarchy_volume = tree.current_hierarchy_volume();
        spdlog::debug("Decomposing level: {0} with {1} max gain and a hierarchy volume {2}", depth, target_settings.gain_threshold, current_hierarchy_volume / initial_bbox.volume());

        for(auto& fas_node : tree.getNodes(depth)) {   //std::for_each(all_parents.begin(), all_parents.end(), [&tree, bbox_algorithm, depth, &node_counter](auto& fas_node){
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

                TwoSplitStrategy twoSplitStrategy;
                auto raster = Discretization::create_discretization(twoSplitStrategy, bbox, coordinate_system2D, 512);
                std::vector<Point2D> projected_2D;
                for(auto& point : fas_node.points){
                    auto proj = projection_plane.projection(std::get<0>(point)); //project to plane (3D)
                    auto point2d = coordinate_system2D.project_onto_plane(proj); //project to plane (2D) & new coordinate system
                    raster.insert(point2d);
                }
#if false
                std::vector<Point> after_transform; // debug
                    auto dim = grid.raster();
                    for(auto i=0; i < dim; ++i){
                        for(auto j=0; j < dim; ++j){
                            if(grid.get(i,j) > 0) {
                                Point2D p2d_grid;
                                p2d_grid << i, j;
                                auto p2d = grid.from_grid(p2d_grid);
                                auto gp = coordinate_system2D.project_to_global(p2d);
                                after_transform.push_back(gp);
                            }
                        }
                    }

                    std::filesystem::path grid_global_path("./grid_global_" + std::to_string(depth) + "_" + std::to_string(node_counter) + "_" + std::to_string(face) + ".csv");
                    if (!CGAL::IO::write_OFF(grid_global_path, after_transform, CGAL::parameters::stream_precision(6))) {
                        std::cout << "oh no\n";
                    }

                    // go primary & secondary direction
#endif
                auto cell_depth = bbox.get_depth(current_face);
                best_bbox_splits[current_face] = raster.best_splits();
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
            // create cutting plane
            Plane_3 projection_plane = bbox.get_plane(selected_face);
            // save coordinate system in super structure ! This should be omitted as its only created for one call;

            auto coordinate_system2D = bbox.get_plane_coordinates2D(selected_face);
            auto p2 = coordinate_system2D.project_to_global(best_split.cut.end);
            auto o1 = coordinate_system2D.project_to_global(best_split.cut.start);
            auto p1 = coordinate_system2D.project_to_global(best_split.origin);
            auto orthogal_vector = CGAL::cross_product(p1 - o1, p2 - o1);
            Plane_3 cutting_plane(o1, p2, o1 + orthogal_vector);
            // TODO: Check if orienation makes sense with ::Y ?
            auto[plane_mesh, plane_system] = Plane2Mesh(o1, p1, p2, best_split.orientation == GridOrientation::Y);

            // write final plane to .off
#ifdef false
            std::filesystem::path cpf( "./final_plane" + std::to_string(depth) + "_" + std::to_string(node_counter++) + "_" +
                            std::to_string(selected_face) + ".off");
                    if (!CGAL::IO::write_OFF(cpf, plane_mesh, CGAL::parameters::stream_precision(6))) {
                        std::cout << "The numer of vertices is: " << plane_mesh.vertices().size() << "\n";
                        throw std::runtime_error("not found");
                    }
#endif

            // split and fit new boxes
            // TODO Move this inside FitAndSplitHierachy !
            auto second_bbox_begins_at = std::partition(fas_node.points.begin(), fas_node.points.end(),
                                                        [&cutting_plane](auto &point) {
                                                            return cutting_plane.has_on_negative_side(std::get<0>(point));
                                                        });

            // calculate new bounding boxes if the points are higher thatn X
            // TODO use ranges !!!
            TPointCloudType vector_bbox1(fas_node.points.begin(), second_bbox_begins_at);
            TPointCloudType vector_bbox2(second_bbox_begins_at, fas_node.points.end());

            BBox bbox1;
            BBox bbox2;

            // Reject if gain or points are not enough
            std::string status = "rejected";
            float gain = -1;
            if(vector_bbox1.size() > target_settings.minimum_point_per_box && vector_bbox2.size() > target_settings.minimum_point_per_box) {
                spdlog::debug("First Box: {}", vector_bbox1.size());
                bbox1 = bbox_algorithm->fit_bounding_box(vector_bbox1);

                spdlog::debug("Second Box: {}", vector_bbox2.size());
                bbox2 = bbox_algorithm->fit_bounding_box(vector_bbox2);
                gain = (current_hierarchy_volume - fas_node.bounding_box.volume() + bbox1.volume() + bbox2.volume()) / current_hierarchy_volume;
                //calculate gain
                if(gain < target_settings.gain_threshold && bbox2.volume() > initial_bbox.volume() / target_settings.minimal_initial_volume_divider && bbox2.volume() > initial_bbox.volume() / target_settings.minimal_initial_volume_divider){
                    tree.emplace_back(depth+1, bbox1, fas_node.points.begin(), second_bbox_begins_at);
                    tree.emplace_back(depth+1, bbox2, second_bbox_begins_at, fas_node.points.end());
                    status = "accepted";
                }
                else{
                    //  reject and not push them to the tree e.g. take the parent
                    fas_node.final = true;
                }
            }
            else{
                // reject and not push them to the tree
                fas_node.final = true;
            }
            // print statistics
            spdlog::trace("Box: {0}\t Points: {1}\t=>\t({2}\t|\t{3})\t gain: {4}\t status: {5}", node_counter, fas_node.points.size(), vector_bbox1.size(), vector_bbox2.size(), gain, status);
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

Target_Setting::Target_Setting(uint32_t kappa_, float gain_threshold_, uint32_t minimum_point_per_box_,
                               uint32_t minimal_initial_volume_divider_) :
        kappa(kappa_), gain_threshold(gain_threshold_), minimum_point_per_box(minimum_point_per_box_), minimal_initial_volume_divider(minimal_initial_volume_divider_){}
