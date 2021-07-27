
#include "Evaluator.h"
#include "helpers.h"
#include "pybind_helpers.h"
#include <execution>
#include <algorithm>
#include <array>


#include <fstream>
#include <iostream>
#include <map>
#include <unordered_map>
#include <utility>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/bounding_box.h>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Surface_mesh.h>


typedef double                     FT;
typedef CGAL::Simple_cartesian<FT> K;
typedef K::Point_2                 Point_2;
typedef K::Point_3                 Point_3;
typedef CGAL::Surface_mesh<Point>  Surface_mesh;



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
    auto points_from_file = helpers::read_file<4>(_pointcloud_src);
//    _pointcloud.reserve(points_from_file.size());
//    if(points_from_file[0][0] == "#") {
//        if (hash_key.length() > 0) {
//            if (hash_key != points_from_file[0][1]) {
//                std::cout << "File missmatch : pointcloud not working for this object_list" << "\n";
//                return false;
//            }
//        } else {
//            hash_key = points_from_file[0][1];
//        }
    std::cout << "Loading Points from"  << _pointcloud_src << "\n";
    std::for_each(points_from_file.cbegin(), points_from_file.cend(), [this](auto const &obj) {
        auto p = Point(std::stof(obj[0]), std::stof(obj[1]), std::stof(obj[2]));
        auto i = std::stoi(obj[3]);
        _pointcloud.emplace_back(std::forward_as_tuple(p,i));
    });
    // sort everything by dimension type => x => y => z
    std::sort( _pointcloud.begin(), _pointcloud.end(), helpers::xyzc_compare);
    std::cout << _pointcloud.size() << " Points have been added to this scene\n";
    return true;
}

bool MvbbEvaluator::load_objects() {
    auto objects = helpers::read_file<2>(_objectlist_src);
    _objectlist.reserve(objects.size());
//    if(objects[0][0] == "#") {
//        if(hash_key.length() > 0){
//            if(hash_key != objects[0][1]) {
//                std::cout << "File missmatch : objectlist not working for this pointcloud" << "\n";
//                //return false;
//            }
//        }
//        else {
//            hash_key = objects[0][1];
//        }
    for(auto const &obj : helpers::range(objects.begin(), objects.end())){
        int id = std::stoi(obj[0]);
        auto obj_name = obj[1];
        boxy::pointcloud_xyzc tmp = this->copy_object_points(id);
        auto box_entity = BoxEntity(static_cast<int>(id), obj_name, tmp);
        _objectlist.insert_or_assign(id, box_entity);
    };
    return true;
}

boxy::pointcloud_xyzc& MvbbEvaluator::get_pointcloud(){
    return _pointcloud ;//helpers::xyzc_2_numpy(_pointcloud);
}
/// This should maybe throw if no data is loaded yet.
/// TODO change logic somehow.
/// \return
objectlist MvbbEvaluator::get_objectlist() {
    return _objectlist;
}

BoxEntity& MvbbEvaluator::get_object(uint32_t object_id) {
    if (!_objectlist.count(object_id)) {
        throw std::runtime_error("This id is not present in your dataset");
    }
    return _objectlist.at(object_id);
}
pointcloud_xyzc MvbbEvaluator::copy_object_points(uint32_t object_id){
    auto tmp_comp =  XYZC {Point(), object_id};
    pointcloud_xyzc tmp;
    std::copy_if (_pointcloud.begin(), _pointcloud.end(), std::back_inserter(tmp), [object_id](auto& point){return std::get<1>(point) == object_id;});
    return tmp;
    return pointcloud_xyzc(std::lower_bound(_pointcloud.begin(), _pointcloud.end(), tmp_comp, helpers::xyzc_objecttype_compare),
                      std::upper_bound(_pointcloud.begin(), _pointcloud.end(), tmp_comp, helpers::xyzc_objecttype_compare));
}

boxy::BBox MvbbEvaluator::bounding_box() const { //pointcloud_xyzc input
    auto start = std::chrono::high_resolution_clock::now();
    std::array<Point, 8> obb_points;
    CGAL::oriented_bounding_box(_pointcloud, obb_points,  CGAL::parameters::use_convex_hull(true).point_map(boxy::Point_map())); //CP::geom_traits(PPoint)
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Elapsed time: " << elapsed.count() << " s\n";
    boxy::BBox bbox(obb_points);
    return bbox;
}

bool MvbbEvaluator::populate(pointcloud_xyzc pcs, std::unordered_map<int, std::string> obj_name_map) {
    _pointcloud = std::move(pcs);
    for (auto const &obj : obj_name_map) {
        int id = obj.first;
        auto obj_name = obj.second;
        boxy::pointcloud_xyzc tmp = this->copy_object_points(id);
        auto box_entity = BoxEntity(id, obj_name, tmp);
        _objectlist.insert_or_assign(id, box_entity);
    };
    return true;
}

MvbbEvaluator create_mvbbevaluator(const std::string &point_src, const std::string &class_src) {
    auto self = MvbbEvaluator();
    self.set_pointcloud_path(point_src);
    self.load_points();
    self.set_objectlist_path(class_src);
    self.load_objects();
    return self;
}
