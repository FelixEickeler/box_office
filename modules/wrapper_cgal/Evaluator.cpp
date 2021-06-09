
#include "Evaluator.h"
#include "helpers.h"
#include <iostream>
#include <algorithm> // for copy
#include <iterator>

#include <unordered_map>

MvbbEvaluator::MvbbEvaluator() = default;

void MvbbEvaluator::set_pointcloud_path(const std::string& _path) {
    _pointcloud_src = Path(_path);
}

std::string MvbbEvaluator::get_pointcloud_path() const{
    return _pointcloud_src;
}

void MvbbEvaluator::set_objectlist_path(const std::string& _path) {
    _objectlist_src = Path(_path);
}

std::string MvbbEvaluator::get_objectlist_path() const{
    return _objectlist_src;
}

bool MvbbEvaluator::load_points() {
    return false;
}

bool MvbbEvaluator::load_objects() {
    auto objects = helpers::read_file<2>(_objectlist_src);
    _objectlist.reserve(objects.size());
    if(objects[0][0] == "#") {
        hash_key = objects[0][1];
        std::for_each(objects.cbegin() + 1, objects.cend(), [this](auto const &obj) {
            _objectlist.insert_or_assign(std::stoi(obj[0]), helpers::trim_copy(obj[1]));
        });
        return true;
    }
    return false;
}

boxy::np_array MvbbEvaluator::get_pointcloud() {
    return helpers::xyzc_2_numpy(_pointcloud);
}

boxy::objectlist MvbbEvaluator::get_objectlist() {
    return _objectlist;
}


