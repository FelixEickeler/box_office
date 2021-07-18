//
// Created by felix on 18.07.2021.
//
#include "mvbb_algorithm.h"
using namespace mvbb;

FitAndSplitHierarchy<TPointCloudType> mvbb::decompose3D(TPointCloudType &points3D, Algo_Base<TPointCloudType> *const bbox_algorithm, uint32_t kappa, float gain_threshold) {

    auto initial_bbox = bbox_algorithm->fit_bounding_box(points3D);
    FitAndSplitHierarchy<TPointCloudType> tree;
    tree.emplace_back(0, initial_bbox, points3D.begin(), points3D.end());

    for(auto depth=0; depth<kappa; ++depth){
        int node_counter = 0;
//            std::for_each(all_parents.begin(), all_parents.end(),
//            [&tree, bbox_algorithm, depth, &node_counter](auto& fas_node){
        if(tree.depth() <= depth) break;
        auto& cur_hierarchy_nodes = tree.getNodes(depth);
        float cur_hierarchy_volume = std::accumulate(cur_hierarchy_nodes.begin(), cur_hierarchy_nodes.end(), 0.0f, [](float sum, auto& node){return node.bounding_box.volume() + sum;});
        std::cout << "hierarchy_volume:" << cur_hierarchy_volume / initial_bbox.volume()<< "\n";
        for(auto& fas_node : cur_hierarchy_nodes) {
            auto bbox = fas_node.bounding_box;
            std::array<BestSplit, 6> all_splits;
            // A=0; B=1; C=2 => see typdef::BoxFaces;
            // calculate each split for evey direction on evey face 3 faces x 2
            for (auto face = 0; face < 3; ++face) {
                auto current_face = static_cast<boxy::BoxFaces>(face);
                auto projection_plane = bbox.get_plane(current_face);
                auto coordinate_system2D = bbox.get_plane_coordinates2D(current_face);
                Rasterizer<512> grid(mvbb::minmax2D(bbox, coordinate_system2D));

                std::vector<Point2D> projected_2D;
//                    projected_2D.reserve(fas_node.points.size());

                // project, change coordinate system and rasterize !
                for(auto& point : fas_node.points){ // = fas_node.points.begin(); point != fas_node.points.end(); ++point) {
//                    std::for_each(fas_node.points.cbegin(), fas_node.points.cend(),
//                                  [projection_plane, coordinate_system2D, &grid](const auto &point) {
                    auto proj = projection_plane.projection(std::get<0>(point)); //project to plane (3D)
                    auto point2d = coordinate_system2D.project_onto_plane(proj); //project to plane (2D) & new coordinate system
                    //care not threading safe
                    grid.insert(point2d);
//                                  });
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
                // TODO this is quickfix and needs to be changed, grid area ?
                auto cell_depth = bbox.get_depth(current_face);
                all_splits[face * 2 + 0] = grid.best_split<direction_ex>(cell_depth);
                all_splits[face * 2 + 0].area /= grid.area();
                all_splits[face * 2 + 1] = grid.best_split<direction_ey>(cell_depth);
                all_splits[face * 2 + 1].area /= grid.area();
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

            // select_best_split
            auto best_split = std::min_element(all_splits.begin(), all_splits.end(),
                                               [](const auto &a, const auto &b) {
                                                   return a.area < b.area;
                                               });

            auto selected_face = static_cast<BoxFaces>(std::distance(all_splits.begin(), best_split) / 2);

            // create cutting plane
            Plane_3 projection_plane = bbox.get_plane(selected_face);
            // save coordinate system in super structure ! This shouldd be omited as its only created for one call;
            auto coordinate_system2D = bbox.get_plane_coordinates2D(selected_face);
            auto p2 = coordinate_system2D.project_to_global(best_split->end_cut);
            auto o1 = coordinate_system2D.project_to_global(best_split->begin_cut);
            auto p1 = coordinate_system2D.project_to_global(best_split->origin);
            auto orthogal_vector = CGAL::cross_product(p1 - o1, p2 - o1);
            Plane_3 cutting_plane(o1, p2, o1 + orthogal_vector);
            auto[plane_mesh, plane_system] = Plane2Mesh(o1, p1, p2, std::distance(all_splits.begin(), best_split) % 2);

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
            if(vector_bbox1.size() > 4 && vector_bbox1.size() > 4) {
                bbox1 = bbox_algorithm->fit_bounding_box(vector_bbox1);
                bbox2 = bbox_algorithm->fit_bounding_box(vector_bbox2);
            }
            else{
                fas_node.final = true;
                // this will reject and not push them to the tree
                continue;
            }
            //calculate gain
            auto gain = (cur_hierarchy_volume - fas_node.bounding_box.volume() + bbox1.volume() + bbox2.volume()) / cur_hierarchy_volume;
            std::string status = "rejected";

            if(gain < gain_threshold){
                tree.emplace_back(depth+1, bbox1, fas_node.points.begin(), second_bbox_begins_at);
                tree.emplace_back(depth+1, bbox2, second_bbox_begins_at, fas_node.points.end());
                status = "accepted";
            }
            else{
                fas_node.final = true;
            }

            std::cout << "\tLevel: " << depth << "\tbox: " << node_counter++ << "\tgain: " << gain << "\t"<< status << "\n";
        }
    }
    // finalize
    for(auto& node : tree.max_depth()){
        node.final = true;
    }
    return tree;
}