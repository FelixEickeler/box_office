#ifndef MVBB_EVALUATOR_H
#define MVBB_EVALUATOR_H

#include <filesystem>
#include <string>
using Path = std::filesystem::path;

class MvbbEvaluator{
    Path _pointcloud_src = "";
    Path _objectlist_src = "";

    public:
        MvbbEvaluator();
        void set_pointcloud_path(std::string _path);
        std::string get_pointcloud_path();
        void set_objectlist_path(std::string _path);
        std::string get_objectlist_path();
}

PYBIND11_MODULE(mvbb, handle){
    handle.doc() = "This module computes the MVBB Decomposition";
    handle.def("get_pointcloud", &get_pointcloud);
//    handle.def("get_pointcloud", &get_pointcloud, py::return_value_policy::move);
}


#endif //MVBB_EVALUATOR_H