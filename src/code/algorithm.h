//
// Created by felix on 18.06.2021.
//

#ifndef TEST_BIND_ALGORITHM_H
#define TEST_BIND_ALGORITHM_H
#include "typedefs.h"
#include <functional>
#include <algorithm>

using namespace boxy;
const bool direction_ex = false;
const bool direction_ey = true;

namespace algorithm {

    //sexy
    template<typename Tpointcloud>
    struct FitAndSplitNode{
        BBox bounding_box;
        float volume{};
        crange<Tpointcloud> elements;
    };

    template<class Tnode_type>
    struct FitAndSplitHierarchy{
        std::vector<Tnode_type> flat_hierarchy;
        boxy::crange<Tnode_type> getNodes(uint32_t kappa){
            uint32_t start_index;
            uint32_t end_index;
            if(kappa > 1) {
                kappa -= 2; // shift
                uint32_t nr_bboxes = 1 << kappa;
                start_index = 1 + 6 * (nr_bboxes >> 1) - 1;
                end_index = start_index * 2 + 5;
            }
            else{
                start_index = 0;
                end_index = 1;
            }
            return boxy::crange<Tnode_type>{flat_hierarchy.begin() + start_index, flat_hierarchy.begin() + end_index};
        }
    };

    template<size_t TRaster>
    class Rasterizer{
        Eigen::Matrix<uint32_t , 2, TRaster> grid;
        std::array<float, 4> min_max;
//        float bias_x;
//        float bias_y;
        float step_x;
        float step_y;

        public:
            explicit Rasterizer(std::array<float,4> _min_max) : min_max{_min_max},
                                                    step_x{(_min_max[2]-min_max[0])/TRaster},
                                                    step_y{(_min_max[3]-min_max[1])/TRaster}{ }

            size_t raster(){
                    return TRaster;
                }

            void insert(Point2D point2D){
                auto x_coord = static_cast<uint32_t>((point2D.x() - min_max[0]) / step_x);
                auto y_coord = static_cast<uint32_t>((point2D.y() - min_max[2]) / step_y);
                grid[x_coord, y_coord] += 1;
            }


            // This method does not distinguish between row and column layout !
           auto get_first_and_last_rowindex(Eigen::VectorXf col){
                struct low_high{
                    uint32_t low=0;
                    uint32_t high=255;
                };

               low_high lh;
               //no abort condition as we dont know if all grid cells are populated
               for(auto row=0; row < TRaster; ++row){
                    if(col(row) != 0){
                        if(lh.low == 0)  lh.low = row;
                        lh.high = row;
                    }
                }
               return lh;
            }

            // This method is calculating the splits on the grid inputet. The result will be gird independent
            // but not transformed in an overall coordinate system.
            auto min_split(){
                uint32_t idx_min_area = 0;
                uint32_t size_min_area = UINT32_MAX;
                for(auto col = 0; col < TRaster;++col){
                    auto[ low, high] = get_first_and_last_rowindex(grid.col(col));
                    // sizer of the boxes is (0->high) * (0->col) + (low->max) * (col->max)
                    auto area = high * col + (TRaster - low) * (TRaster-col);
                    if(area < size_min_area){
                        idx_min_area = col;
                        size_min_area = area;
                    }
                }
                return BestGridSplit{size_min_area, idx_min_area};
            }

            // direction 0 => first orientation, direction true(1) => second orientation:
            // This is a little hacky but is my first constexpr !
            // Ah btw this is not transformed in the a real coordinate system (e.g. cutting plane)
            template<bool direction>
            BestSplit best_split(){
                auto bgs = min_split();
                float cell_area = step_x * step_y;
                Vector2f cutting_point;
                if constexpr (direction){
                    cutting_point << 0, bgs.index * step_y + min_max[1];
                }
                else{
                    cutting_point << bgs.index * step_x + min_max[0], 0;
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

    struct Algo_MVBB{};
    struct Algo_PCA{};

    template<class TPointCloudType>
    BBox fit_bounding_box(TPointCloudType points3D, Algo_MVBB algo);     // Declare, but don't define

    template<class TPointCloudType>
    BBox fit_bounding_box(TPointCloudType points3D, Algo_PCA algo);


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
        std::function<BBox(TPointCloudType)> fit_bbox;
        switch(algorithm){
            case BoundingBoxAlgorithm::MVBB:
                fit_bbox = [](TPointCloudType pt3D){return fit_bounding_Box(pt3D, Algo_MVBB());};
                break;
            case BoundingBoxAlgorithm::PCA:
                fit_bbox = [](TPointCloudType pt3D){return fit_bounding_Box(pt3D, Algo_PCA());};
                break;
        }

        auto initial_bbox = fit_bbox(points3D);
        FitAndSplitHierarchy<TPointCloudType> tree;
        tree.flat_hierachry.emplace(initial_bbox);

        for(auto depth=0; depth<kappa; ++depth){
            auto all_parents =  tree.getNodes(depth);
            std::for_each(all_parents.cbegin(), all_parents.cend(),
            [points3D, tree, fit_bbox](auto fas_node){
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
                auto second_bbox_begins_at = std::partition(points3D.cbegin(), points3D.cend(),
                                                         [cutting_plane](auto& point){cutting_plane.has_on_negative_side(point);});


                BBox bbox1 = fit_bbox(TPointCloudType(points3D.begin(), second_bbox_begins_at));
                BBox bBox2 = fit_bbox(TPointCloudType(second_bbox_begins_at, points3D.end()));

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