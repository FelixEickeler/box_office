//
// Created by felix on 18.06.2021.
//

#ifndef TEST_BIND_ALGORITHM_H
#define TEST_BIND_ALGORITHM_H
#include "typedefs.h"
#include <functional>
#include <algorithm>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/bounding_box.h>
#include <iterator>
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
        VectorView<typename Tpointcloud::const_iterator> elements;
    };

    template<class TNodeType>
    struct FitAndSplitHierarchy{
        std::vector<TNodeType> flat_hierarchy;
        boxy::VectorView<typename std::vector<TNodeType>::const_iterator> getNodes(uint32_t kappa){
            uint32_t start_index;
            uint32_t end_index;
            if(kappa > 1) {
                kappa -= 1; // shift1
                uint32_t nr_bboxes = 1 << kappa;
                start_index = nr_bboxes;
                end_index = start_index * 2 - 1;
            }
            else{
                start_index = 0;
                end_index = 1;
            }
            return boxy::VectorView(flat_hierarchy.cbegin() + start_index, flat_hierarchy.cbegin() + end_index);
        }
    };

    template<size_t TRaster>
    class Rasterizer{
        MatrixXu grid = MatrixXu::Zero(TRaster, TRaster);
//        Eigen::Matrix<uint32_t ,TRaster,TRaster> grid = Eigen::Matrix<uint32_t, TRaster,TRaster>::Zero();//(TRaster, TRaster);

        std::array<float, 4> min_max;
        float step_x;
        float step_y;

        public:
            explicit Rasterizer(std::array<float,4> _min_max) : min_max{_min_max},
                                                    step_x{(_min_max[2]-min_max[0])/(TRaster-1)},
                                                    step_y{(_min_max[3]-min_max[1])/(TRaster-1)}{ }

            [[nodiscard]] size_t raster() const{
                    return TRaster;
                }

            [[nodiscard]] std::tuple<float, float> step_sizes() const{
                return {step_x, step_y};
            }

            [[nodiscard]] const auto& get_grid() const{
                return grid;
            }

            void insert(Point2D point2D){
                auto x_coord = static_cast<uint32_t>((point2D.x() - min_max[0]) / step_x);
                auto y_coord = static_cast<uint32_t>((point2D.y() - min_max[1]) / step_y);
                grid(y_coord, x_coord) += 1;
            }

            /// This method returns a vector of spans represneting the occupoied space of the grid.
            /// It will be either provide these as column : direciton_x(false)  or row : direction_y(true) represntations
            /// \tparam TDirection :  true for row; false for column
            /// \return Eigen::Vector2i<TRaster : v(n,0) start of the span and v(n,1) end of it
           template<bool TDirection>
           auto get_first_and_last_slot(){
                MatrixXu _span = MatrixXu::Zero(2, TRaster);

               for(auto i=0; i < TRaster; ++i){
                   bool first_set = false;
                   for(auto j=0; j < TRaster; ++j){
                        if constexpr(TDirection){
                           if(grid(i, j) != 0){
                               if(!first_set){
                                   _span(0, i) = j;
                                   first_set = true;
                               }
                               _span(1, i) = j+1;
                            }
                        }
                        else{
                            if(grid(j, i) != 0){
                                if(!first_set){
                                    _span(0, i) = j;
                                    first_set = true;
                                }
                                _span(1, i) = j+1;
                            }
                        }
                   }
                }
               return _span;
            }


            /// This method is calculating the splits on the grid of the rasterizer. It is not transformed in an overall coordinate system !
            /// \tparam TDirection :  true for row; false for column
            /// \return BestGridSplit : rasterized position, no global space !
           template <bool TDirection>
           auto best_split_rasterized(){
              auto slot_sizes = get_first_and_last_slot<TDirection>();
              MatrixXu zero_stopper = (slot_sizes.row(1).array() == 0).template cast<uint32_t>()*UINT32_MAX;
              zero_stopper += slot_sizes.row(0);
              Eigen::Matrix<uint32_t, 2, 2> bbox_boundaries = Eigen::Matrix<uint32_t, 2,2>::Zero();

               auto min_area = UINT32_MAX;
               uint32_t idx_min_area = 0;
//               auto test = slot_sizes.block(0,0,2,TRaster);


               // cut between 0 and 1 of grid, last row does not after
               for(auto idx=0; idx < slot_sizes.cols()-1; ++idx){
                   uint32_t wbbox2 = TRaster-idx-1;
                   if(bbox_boundaries(0,0) > slot_sizes(0,idx)) bbox_boundaries(0,0) = slot_sizes(0,idx);
                   if(bbox_boundaries(1,0) < slot_sizes(1,idx)) bbox_boundaries(1,0) = slot_sizes(1,idx);

                   // only evaluate non zero:zero columns, for minimum => zero_stopper!
                   bbox_boundaries(0,1) = zero_stopper.topRightCorner(1,wbbox2).minCoeff();
                   bbox_boundaries(1,1) = slot_sizes.bottomRightCorner(1,wbbox2).maxCoeff();

                   auto area = (idx+1) * (bbox_boundaries(1,0) -  bbox_boundaries(0,0))+
                           wbbox2 * (bbox_boundaries(1,1) -  bbox_boundaries(0,1));

                   if(area < min_area){
                       min_area = area;
                       idx_min_area = idx;
                   }
               }
               return BestGridSplit{min_area, idx_min_area};
           }

            // direction 0 => first orientation, direction true(1) => second orientation:
            // This is a little hacky but is my first constexpr !
            // Ah btw this is not transformed in the a real coordinate system (e.g. cutting plane)
            template<bool TDirection>
            BestSplit best_split(){
                auto bgs = best_split_rasterized<TDirection>();
                float cell_area = step_x * step_y;

                Vector2f cutting_point;
                if constexpr (TDirection){ // true being direction_ey
                    cutting_point << 0, (bgs.index+1) * step_y + min_max[1];
                }
                else{
                    cutting_point << (bgs.index+1) * step_x + min_max[0], 0;
                }
                return BestSplit{bgs.area * cell_area, cutting_point};
            }
    };


//    void best_split_search(grid);

    enum BoundingBoxAlgorithm { MVBB, PCA };
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
            virtual BBox fit_bounding_box(TPointCloudType& points3D) = 0;  // This method is not implemented in the base class, making it a pure virtual method. Subclasses must implement it
        };

    template<class TPointCloudType>
    class Algo_MVBB : Algo_Base<TPointCloudType>{
        public:
            ~Algo_MVBB()= default;
            BBox fit_bounding_box(TPointCloudType& points3D);
    };

    template<class TPointCloudType>
    BBox Algo_MVBB<TPointCloudType>::fit_bounding_box(TPointCloudType& points3D) {
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
            if(y < min_max2D[0]){
                min_max2D[1] = x;
            }
            else if(y > min_max2D[2]){
                min_max2D[3] = x;
            }
        }
        return min_max2D;
    }

    template<class TPointCloudType>
    void decompose3D(TPointCloudType points3D, BoundingBoxAlgorithm algorithm, uint32_t kappa){
        std::unique_ptr<Algo_Base<TPointCloudType>> bbox_algorithm;
        switch(algorithm){
            case BoundingBoxAlgorithm::MVBB:
                bbox_algorithm = new Algo_MVBB<TPointCloudType>();
                break;

//            case BoundingBoxAlgorithm::PCA:
//                bbox_algorithm = new Algo_PCA<TPointCloudType>();
//                break;
        }

        auto initial_bbox = bbox_algorithm(points3D);
        FitAndSplitHierarchy<TPointCloudType> tree;
        tree.flat_hierachry.emplace(initial_bbox);

        for(auto depth=0; depth<kappa; ++depth){
            auto all_parents =  tree.getNodes(depth);
            std::for_each(all_parents.cbegin(), all_parents.cend(),
            [points3D, tree, bbox_algorithm](auto fas_node){
                auto bbox = fas_node.bounding_box;
                std::array<BestSplit, 6> all_splits;
                // A=0; B=1; C=2 => see typdef::BoxFaces;
                // calculate each split for evey direction on evey face 3 faces x 2
                for(auto face=0; face < 3; ++face){
                    auto projection_plane = bbox.get_face(static_cast<boxy::BoxFaces>(face));
                    auto coordinate_system2D = bbox.get_plane_coordinates2D(static_cast<boxy::BoxFaces>(face));
                    Rasterizer<256> grid(minmax2D(bbox, coordinate_system2D));

                    std::vector<Point2D> projected_2D;
                    projected_2D.reserve(points3D.size());

                    // project, change coordinate system and rasterize !
                    std::for_each(points3D.cbegin(), points3D.cend(), [projection_plane, coordinate_system2D, grid](const auto& point){
                        auto proj = projection_plane.projection(point); //project to plane (3D)
                        auto point2d =  coordinate_system2D.project(proj); //project to plane (2D) & new coordinate system
                        //care not threading safe
                        grid.insert(point2d);
                    });

                    // go primary & secondary direction
                    all_splits[face*2+0] = grid.best_split<direction_ex>();
                    all_splits[face*2+1] = grid.best_split<direction_ey>();
                }
                // select_best_split
                auto best_split = std::min_element( all_splits.begin(), all_splits.end(),
                                             []( const auto& a, const auto& b ){
                                                 return a.area < b.area;
                                             } );

                auto selected_face = static_cast<uint8_t>(best_split - all_splits.begin());
                auto projection_plane = bbox.get_face(static_cast<boxy::BoxFaces>(selected_face));
                // this could be omited as its only created for one call;
                auto coordinate_system2D = bbox.get_plane_coordinates2D(static_cast<boxy::BoxFaces>(selected_face));
                auto fulcrum = coordinate_system2D.project_to_global(best_split.coordinate);
                Plane_3 cutting_plane(fulcrum, projection_plane.orthogonal_vector);

                // split and fit new boxes
                // TODO Move this inside FitAndSplitHierachy !
                auto second_bbox_begins_at = std::partition(points3D.cbegin(), points3D.cend(),
                                                         [cutting_plane](auto& point){cutting_plane.has_on_negative_side(point);});


                BBox bbox1 = bbox_algorithm.fit_bounding_box(points3D.begin(), second_bbox_begins_at);
                BBox bBox2 = bbox_algorithm.fit_bounding_box(second_bbox_begins_at, points3D.end());

                // update tree
                tree.flat_hierachry.emplace(bbox1);
                tree.flat_hierachry.emplace(bBox2);
            });
        }
    }
};


#endif //TEST_BIND_ALGORITHM_H
//        std::for_each(points_from_file.cbegin() + 1, points_from_file.cend(), [this](auto const &obj) {
//            auto p = Point(std::stof(obj[0]), std::stof(obj[1]), std::stof(obj[2]));
//            auto i = std::stoi(obj[3]);
//            _pointcloud.emplace_back(std::forward_as_tuple(p,i));
//        });