////
//// Created by felix on 17.07.2021.
////
//#include "typedefs.h"
//#include "mvbb_algorithms.h"
//#include <BoxScene.h>
//#include <spdlog/spdlog.h>
//
//int main(int argc, char* args[]){
//    spdlog::info("Welcome to spdlog version {}.{}.{}  !", SPDLOG_VER_MAJOR, SPDLOG_VER_MINOR, SPDLOG_VER_PATCH);
//    auto pcs_path = "../../../data/in/bunny_classy.txt";
//    auto ol_path  = "../../../data/in/bunny_classy.ol";
//    mvbb::CGAL_MVBB<pointcloud_xyzc> algo;
//    auto scene = create_scene(pcs_path, ol_path);
//
//    std::unordered_map<std::string, std::vector<FitAndSplitNode<pointcloud_xyzc>>> all_boxes;
//    spdlog::set_level(spdlog::level::trace);
//    for(auto& entity : scene.get_objectlist()){
//        spdlog::info("Entity Name: {}", entity.second.get_name());
//        auto boxes = entity.second.decompose(4);
//        all_boxes.insert_or_assign(entity.second.get_name(), boxes);
//        spdlog::set_pattern("%+");
//    }
//    for(auto& b :  all_boxes){
//        spdlog::warn("{} was decomposed in {} boxes !!!", b.first, b.second.size());
//    }
//}