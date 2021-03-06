//
// Created by felix on 28.07.2021.
//

#ifndef BOXOFFICE_FITANDSPLITHIERARCHY_H
#define BOXOFFICE_FITANDSPLITHIERARCHY_H
#include "typedefs.h"
using namespace boxy;


template<typename Tpointcloud>
struct FitAndSplitNode {
    BBox bounding_box;
    VectorView<typename Tpointcloud::iterator> points = nullptr;
    bool final = false;

    explicit FitAndSplitNode(BBox _bounding_box, typename Tpointcloud::iterator begin,
                             typename Tpointcloud::iterator end) : bounding_box(_bounding_box), points(begin, end) {}

    Tpointcloud get_points() const{
        return Tpointcloud(points.begin(), points.end());
    }

    bool operator==(const FitAndSplitNode<Tpointcloud>& that) const{
        return this->points == that.points;
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

        auto node_depth(FitAndSplitNode<TPointCloud> node){
            auto k = 0;
            for (auto& level : nodes){
                if(std::find(begin(level), end(level), node) != end(level)){
                    break;
                }
                ++k;
            }
            return k;
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

        float current_hierarchy_volume(){
            return std::accumulate(nodes.begin(), nodes.end()-1, 0.0f, [](float sum, auto& cur_nodes){
                return std::accumulate(cur_nodes.begin(), cur_nodes.end(), 0.0f, [](float inner_sum, const auto& node){
                    return node.final ? node.bounding_box.volume() + inner_sum : inner_sum;
                }) + sum;
            }) + current_depth_volume();
        }

        float current_depth_volume(){
            return std::accumulate(nodes.back().begin(), nodes.back().end(), 0.0f, [](float sum, auto& node){return node.bounding_box.volume() + sum;});
        }
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

#endif //BOXOFFICE_FITANDSPLITHIERARCHY_H
