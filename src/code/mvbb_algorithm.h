//
// Created by felix on 18.06.2021.
//
#define GTEST_COUT std::cerr << "[          ] [ INFO ]"
#ifndef TEST_BIND_ALGORITHM_H
#define TEST_BIND_ALGORITHM_H
#include "typedefs.h"
#include "helpers.h"
#include "helpers_nobind.h"
#include "rasterizer..h"
#include <functional>
#include <algorithm>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/bounding_box.h>
#include <iterator>
#include <fstream>
#include <CGAL/IO/write_off_points.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Origin.h>

//#include <boost/range>

using namespace boxy;
const bool direction_ex = false;
const bool direction_ey = true;


namespace mvbb {

    //sexy
    template<typename Tpointcloud>
    struct FitAndSplitNode{
        BBox bounding_box;
        float volume{};
        VectorView<typename Tpointcloud::iterator> points;

        explicit FitAndSplitNode(BBox _bounding_box) : bounding_box(_bounding_box){}
        explicit FitAndSplitNode(BBox _bounding_box, typename Tpointcloud::iterator begin, typename Tpointcloud::iterator end) : bounding_box(_bounding_box), points(begin, end){}
    };

    template<class TNodeType>
    struct FitAndSplitHierarchy{
        std::vector<TNodeType> flat_hierarchy;
        boxy::VectorView<typename std::vector<TNodeType>::const_iterator> getNodes(uint32_t kappa){
            uint32_t start_index;
            uint32_t end_index;
            if(kappa >= 1) {
//                kappa -= 1; // shift1
                uint32_t nr_bboxes = 1 << kappa;
                start_index = nr_bboxes-1;
                end_index = start_index+ nr_bboxes;
            }
            else{
                start_index = 0;
                end_index = 1;
            }
            return boxy::VectorView(flat_hierarchy.cbegin() + start_index, flat_hierarchy.cbegin() + end_index);
        }
    };


//    void best_split_search(grid);

//    enum BoundingBoxAlgorithm { MVBB, PCA };
//
//    template<BoundingBoxAlgorithm TAlgo>
//    struct BBoxAlgo{};
//
//    using Algo_MVBB = BBoxAlgo<BoundingBoxAlgorithm::MVBB>;
//    using Algo_PCA = BBoxAlgo<BoundingBoxAlgorithm::MVBB>;


    template<class TPointCloudType>
    class Algo_Base{
        public:
            virtual ~Algo_Base () {};
            using iterator = typename TPointCloudType::iterator;
            virtual BBox fit_bounding_box(TPointCloudType& points3D) const = 0;  // This method is not implemented in the base class, making it a pure virtual method. Subclasses must implement it
        };

    template<class TPointCloudType>
    class Algo_MVBB : public Algo_Base<TPointCloudType>{
        public:
            ~Algo_MVBB()= default;
            BBox fit_bounding_box(TPointCloudType& points3D) const;
    };

    template<class TPointCloudType>
    BBox Algo_MVBB<TPointCloudType>::fit_bounding_box(TPointCloudType& points3D) const {
        BBox tmp;
        // TODO figure out how to generate pass the view to the oriented_bounding_box
        boxy::VectorView input(points3D.begin(), points3D.end());
//        boxy::CMSRange<TPointCloudType> input(points3D.cbegin(), points3D.cend());
//        boost::range:: abc;
//        (points3D.begin(), points3D.end());
        CGAL::oriented_bounding_box(points3D, tmp.vertices,  CGAL::parameters::use_convex_hull(true).point_map(boxy::Point_map())); //CP::geom_traits(PPoint)
        return tmp;
    }
//
//    template<class TPointCloudType>
//    class Algo_PCA : Algo_Base<TPointCloudType>{
//       BBox fit_bounding_box(TPointCloudType& points3D);
//    };
//
//    template<class TPointCloudType>
//    BBox Algo_PCA<TPointCloudType>::fit_bounding_box(TPointCloudType &points3D) {
//        // TODO: write test for pca fit
//        BBox tmp;
//        CGAL::Simple_cartesian<float>::Iso_cuboid_3 c3 = CGAL::bounding_box(points3D->begin(), points3D->end());
//        for(auto i=0; i < 8; ++i){
//            tmp.vertices[i] = c3.vertex(i);
//        }
//        return tmp;
//    }

    std::array<float, 4> minmax2D(const BBox& bbox, const CoordinateSystem2D& coordinate_system2D) {
        std::array<float, 4> min_max2D{1.0e10, -1.0e-10,1.0e-10,-1.0e-10};
        for(auto& v : bbox.vertices){
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

    template<class TPointCloudType>
    FitAndSplitHierarchy<FitAndSplitNode<TPointCloudType>> decompose3D(TPointCloudType& points3D, Algo_Base <TPointCloudType>* const bbox_algorithm, uint32_t kappa){

        auto initial_bbox = bbox_algorithm->fit_bounding_box(points3D);
        FitAndSplitHierarchy<FitAndSplitNode<TPointCloudType>> tree;
        tree.flat_hierarchy.emplace_back(initial_bbox, points3D.begin(), points3D.end());

        for(auto depth=0; depth<=kappa; ++depth){
            auto all_parents =  tree.getNodes(depth);
            int parrent_counter = 0;
            std::for_each(all_parents.begin(), all_parents.end(),
            [&tree, bbox_algorithm, depth, &parrent_counter](auto& fas_node){
                auto bbox = fas_node.bounding_box;
                std::array<BestSplit, 6> all_splits;
                // A=0; B=1; C=2 => see typdef::BoxFaces;
                // calculate each split for evey direction on evey face 3 faces x 2
                for(auto face=0; face < 3; ++face){
                    auto current_face = static_cast<boxy::BoxFaces>(face);
                    auto projection_plane = bbox.get_plane(current_face);
                    auto coordinate_system2D = bbox.get_plane_coordinates2D(current_face);
                    Rasterizer<128> grid(minmax2D(bbox, coordinate_system2D));

                    std::vector<Point2D> projected_2D;
                    projected_2D.reserve(fas_node.points.size());

                    // project, change coordinate system and rasterize !
                    std::for_each(fas_node.points.cbegin(), fas_node.points.cend(), [projection_plane, coordinate_system2D, &grid](const auto& point){
                        auto proj = projection_plane.projection(std::get<0>(point)); //project to plane (3D)
                        auto point2d =  coordinate_system2D.project_onto_plane(proj); //project to plane (2D) & new coordinate system
                        //care not threading safe
                        grid.insert(point2d);
                    });

                    #if true
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
                    std::filesystem::path grid_path("./grid_" + std::to_string(depth) + "_" + std::to_string(face) + ".csv");
                    grid.save_grid(grid_path);

                    std::filesystem::path grid_global_path("./grid_global_" + std::to_string(depth) + "_" + std::to_string(face) + ".csv");
                    if (!CGAL::IO::write_OFF(grid_global_path, after_transform, CGAL::parameters::stream_precision(6))) {
                        std::cout << "oh no\n";
//                        ASSERT_TRUE(false) << "grid_glaobal not written, not sure why, maybe check path: "
//                                           << std::filesystem::current_path();
                    }
                    // go primary & secondary direction
                    #endif
                    // TODO this is quickfix and needs to be changed
                    auto cell_depth = bbox.get_depth(current_face);
                    all_splits[face*2+0] = grid.best_split<direction_ex>(cell_depth);
                    all_splits[face*2+0].area /= grid.area();
                    all_splits[face*2+1] = grid.best_split<direction_ey>(cell_depth);
                    all_splits[face*2+1].area /= grid.area();
                    #if true
                    for(auto i = 0; i < 2; ++i) {
                        auto cut = coordinate_system2D.project_to_global(all_splits[face * 2 + i].end_cut);
                        auto fulcrum = coordinate_system2D.project_to_global(all_splits[face * 2 + i].begin_cut);
                        auto origin = coordinate_system2D.project_to_global(grid.from_grid(Point2D(0,0)));
                        auto[plane_mesh, plane_system] = Plane2Mesh(fulcrum, origin, cut, i > 0);


                        std::filesystem::path cplan_off("./cutting_plane_" + std::to_string(depth) + "_" + std::to_string(face) + "_" + std::to_string(i) + ".off");
                        if (!CGAL::IO::write_OFF(cplan_off, plane_mesh, CGAL::parameters::stream_precision(6))) {
                            std::cout << "oh no2\n";
                        }

                        std::filesystem::path plane_system_path("./cutting_plane_system" + std::to_string(depth) + "_" + std::to_string(face) + "_" + std::to_string(i) + ".off");
                        if (!CGAL::IO::write_OFF(plane_system_path, plane_system, CGAL::parameters::stream_precision(6))) {
                            std::cout << "oh no3\n";
                        }


//                        GTEST_COUT << "CUT: " << depth << "Plane: " << std::to_string(face) << "direction:" << i << " Volumne" << std::to_string(all_splits[face*2+i].area)
//                               << std::endl;
//                }
                    }
                    #endif

                }

                // select_best_split
                auto best_split = std::min_element( all_splits.begin(), all_splits.end(),
                                             []( const auto& a, const auto& b ){
                                                 return a.area < b.area;
                                             } );

                auto lll = 0;
                for(auto& split : all_splits) {
                    GTEST_COUT << "Plane: " << std::to_string(lll++) << " Volumne" << std::to_string(split.area)
                               << std::endl;
                }
                GTEST_COUT << "Volumne of choosen plane: " << std::to_string(best_split->area) << std::endl;

                auto selected_face = static_cast<BoxFaces>(std::distance(all_splits.begin(), best_split)/2);

                Plane_3 projection_plane = bbox.get_plane(selected_face);
                GTEST_COUT << "Face " << std::to_string(selected_face) << std::endl;

                // this could be omited as its only created for one call;
                auto coordinate_system2D = bbox.get_plane_coordinates2D(selected_face);

                auto p2 = coordinate_system2D.project_to_global(best_split->end_cut);
                auto o1  = coordinate_system2D.project_to_global(best_split->begin_cut);
                auto p1 = coordinate_system2D.project_to_global(best_split->origin);
                auto orthogal_vector = CGAL::cross_product(p1- o1,  p2- o1);
                Plane_3 cutting_plane(o1, p2, o1 + orthogal_vector);

//                std::vector<Point> oo = { cutting_plane.point(), cutting_plane.point() + cutting_plane.base1(), cutting_plane.point()+ cutting_plane.base2()};
                auto[plane_mesh, plane_system] = Plane2Mesh(o1, p1, p2, std::distance(all_splits.begin(), best_split) % 2);
//                std::vector<Point> oo = { o1, p2, o1 + orthogal_vector};
                std::filesystem::path cpf("./final_plane" + std::to_string(depth) + "_" + std::to_string(parrent_counter++) + "_" + std::to_string(selected_face) + ".off");
                if (!CGAL::IO::write_OFF(cpf, plane_mesh, CGAL::parameters::stream_precision(6))) {
                    std::cout << "oh no100\n";
                    throw std::runtime_error("not found");
                }

                // split and fit new boxes
                // TODO Move this inside FitAndSplitHierachy !
                auto second_bbox_begins_at = std::partition(fas_node.points.begin(), fas_node.points.end(),
                                     [&cutting_plane](auto& point){return cutting_plane.has_on_negative_side(std::get<0>(point));});

                // TODO use ranges !!!
                TPointCloudType vector_bbox1(fas_node.points.begin(), second_bbox_begins_at);
                BBox bbox1 = bbox_algorithm->fit_bounding_box(vector_bbox1);
                TPointCloudType vector_bbox2(second_bbox_begins_at,fas_node.points.end());
                BBox bBox2 = bbox_algorithm->fit_bounding_box(vector_bbox2);

                // update tree
                tree.flat_hierarchy.emplace_back(bbox1, fas_node.points.begin(), second_bbox_begins_at);
                tree.flat_hierarchy.emplace_back(bBox2, second_bbox_begins_at, fas_node.points.end());
            });
        }
        return tree;
    }
};


#endif //TEST_BIND_ALGORITHM_H
//        std::for_each(points_from_file.cbegin() + 1, points_from_file.cend(), [this](auto const &obj) {
//            auto p = Point(std::stof(obj[0]), std::stof(obj[1]), std::stof(obj[2]));
//            auto i = std::stoi(obj[3]);
//            _pointcloud.emplace_back(std::forward_as_tuple(p,i));
//        });