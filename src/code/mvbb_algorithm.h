//
// Created by felix on 18.06.2021.
//
#define GTEST_COUT std::cerr << "[          ] [ INFO ]"
#ifndef TEST_BIND_ALGORITHM_H
#define TEST_BIND_ALGORITHM_H
#include "typedefs.h"
#include "helpers.h"
#include "rasterizer.h"
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
#include <filesystem>

//#include <boost/range>

using namespace boxy;
const bool direction_ex = false;
const bool direction_ey = true;


namespace mvbb {

    //sexy
    template<typename Tpointcloud>
    struct FitAndSplitNode {
        BBox bounding_box;
        VectorView<typename Tpointcloud::iterator> points = nullptr;
        bool final = false;

//        const NewVectorView points;
//        explicit FitAndSplitNode(BBox _bounding_box) : bounding_box(_bounding_box){}
        explicit FitAndSplitNode(BBox _bounding_box, typename Tpointcloud::iterator begin,
                                 typename Tpointcloud::iterator end) : bounding_box(_bounding_box), points(begin, end) {}

        Tpointcloud get_points() const{
            return Tpointcloud(points.begin(), points.end());
        }
    };

    template<class TPointCloud>
    class FitAndSplitHierarchy {
//        std::vector<TNodeType> flat_hierarchy;
            std::vector<std::vector<FitAndSplitNode<TPointCloud>>> nodes;
        public:
            [[nodiscard]] std::vector<FitAndSplitNode<TPointCloud>> &getNodes(uint32_t kappa) {
                if (kappa >= nodes.size()) {
                    throw std::runtime_error("The requested depth does not exist");
                }
                return nodes[kappa];
            }

            auto emplace_back(uint32_t level, BBox _bounding_box, typename TPointCloud::iterator begin, typename TPointCloud::iterator end) {
                if (level >= nodes.size()) {
                    nodes.emplace_back();
                }
                return nodes[level].emplace_back(_bounding_box, begin, end);
            }

            size_t depth() {
                return nodes.size();
            }

            auto &max_depth() {
                return nodes.back();
            }

            std::vector<FitAndSplitNode<TPointCloud>> get_finalized(int node_level = -1){
                if(node_level < 0) node_level = depth();
                std::vector<FitAndSplitNode<TPointCloud>> collection;
                for(int k =0; k < node_level; ++k) {
                    uint32_t sk = 0;
                    for (auto &node : getNodes(k)) {
                        if (node.final) {
                            collection.push_back(node);
                        }
                    }
                }
                return collection;

            }

//        boxy::VectorView<typename std::vector<TNodeType>::const_iterator> getNodes(uint32_t kappa){
//            uint32_t start_index;
//            uint32_t end_index;
//            if(kappa >= 1) {
////                kappa -= 1; // shift1
//                uint32_t nr_bboxes = 1 << kappa;
//                start_index = nr_bboxes-1;
//                end_index = start_index+ nr_bboxes;
//            }
//            else{
//                start_index = 0;
//                end_index = 1;
//            }
//            return boxy::VectorView(flat_hierarchy.cbegin() + start_index, flat_hierarchy.cbegin() + end_index);
//        }

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
    class Algo_Base {
        public:
            virtual ~Algo_Base() {};
            using iterator = typename TPointCloudType::iterator;

            virtual BBox fit_bounding_box(
                    TPointCloudType &points3D) const = 0;  // This method is not implemented in the base class, making it a pure virtual method. Subclasses must implement it
    };

    template<class TPointCloudType>
    class Algo_MVBB : public Algo_Base<TPointCloudType> {
        public:
            ~Algo_MVBB() = default;

            BBox fit_bounding_box(TPointCloudType &points3D) const;
    };

    template<class TPointCloudType>
    BBox Algo_MVBB<TPointCloudType>::fit_bounding_box(TPointCloudType &points3D) const {
        // TODO figure out how to generate pass the view to the oriented_bounding_box
        boxy::VectorView input(points3D.begin(), points3D.end());
        std::array<Point, 8> vertices;
        if (points3D.size() > 10) {
            try {
                CGAL::oriented_bounding_box(points3D, vertices, CGAL::parameters::point_map(boxy::Point_map())); //CGAL::parameters::use_convex_hull(true).
            }
            catch (std::exception &e) {
                std::cout << e.what() << '\n';
            }
        }
        return BBox(vertices);
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



//    template<class TPointCloudType>
    using TPointCloudType = pointcloud_xyzc;
    FitAndSplitHierarchy<TPointCloudType>
    decompose3D(TPointCloudType &points3D, Algo_Base<TPointCloudType> *const bbox_algorithm, uint32_t kappa, float gain_threshold = 0.99);

}
#endif