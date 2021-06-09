#ifndef MVBB_EVALUATOR_H
#define MVBB_EVALUATOR_H
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>


#include <filesystem>
#include <string>
#include "typedefs.h"
using Path = std::filesystem::path;

class MvbbEvaluator{
    Path _pointcloud_src = "";
    Path _objectlist_src = "";
    boxy::pointcloud_xyzc _pointcloud;
    boxy::objectlist _objectlist;
    std::string hash_key = "";

    public:
        MvbbEvaluator();
        void set_pointcloud_path(const std::string& _path);
        std::string get_pointcloud_path() const;
        void set_objectlist_path(const std::string& _path);
        std::string get_objectlist_path() const;
        bool load_points();
        bool load_objects();

        boxy::np_array get_pointcloud();
        boxy::objectlist get_objectlist();
};




#endif //MVBB_EVALUATOR_H