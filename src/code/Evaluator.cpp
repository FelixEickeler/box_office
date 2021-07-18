
#include "Evaluator.h"
#include "helpers.h"
#include <execution>
#include <algorithm>
#include <array>


#include <fstream>
#include <iostream>
#include <map>
#include <unordered_map>

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
    _pointcloud.reserve(points_from_file.size());
    if(points_from_file[0][0] == "#") {
        if (hash_key.length() > 0) {
            if (hash_key != points_from_file[0][1]) {
                std::cout << "File missmatch : pointcloud not working for this object_list" << "\n";
                return false;
            }
        } else {
            hash_key = points_from_file[0][1];
        }
        std::for_each(points_from_file.cbegin() + 1, points_from_file.cend(), [this](auto const &obj) {
            auto p = Point(std::stof(obj[0]), std::stof(obj[1]), std::stof(obj[2]));
            auto i = std::stoi(obj[3]);
            _pointcloud.emplace_back(std::forward_as_tuple(p,i));
        });
        // sort everything by dimension type => x => y => z
        std::sort(std::execution::par_unseq, _pointcloud.begin(), _pointcloud.end(), helpers::xyzc_compare);
        return true;
    }
    return false;
}

bool MvbbEvaluator::load_objects() {
    auto objects = helpers::read_file<2>(_objectlist_src);
    _objectlist.reserve(objects.size());
    if(objects[0][0] == "#") {
        if(hash_key.length() > 0){
            if(hash_key != objects[0][1]) {
                std::cout << "File missmatch : objectlist not working for this pointcloud" << "\n";
                return false;
            }
        }
        else {
            hash_key = objects[0][1];
        }
        std::for_each(objects.cbegin() + 1, objects.cend(), [this](auto const &obj) {
        _objectlist.insert_or_assign(std::stoi(obj[0]), obj[1]);
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

boxy::VectorView<pointcloud_xyzc::const_iterator> MvbbEvaluator::get_object(uint32_t object_id) const {
    auto tmp_comp =  XYZC {Point(), object_id};
    return VectorView(std::lower_bound(_pointcloud.begin(), _pointcloud.end(), tmp_comp, helpers::xyzc_objecttype_compare),
                      std::upper_bound(_pointcloud.begin(), _pointcloud.end(), tmp_comp, helpers::xyzc_objecttype_compare));
}

boxy::BBox MvbbEvaluator::bounding_box() const { //pointcloud_xyzc input
    auto start = std::chrono::high_resolution_clock::now();
//    std::vector<Point_3> points_3;
//    points_3.push_back(Point_3(1.0, 0.0, 0.5));
//    points_3.push_back(Point_3(2.0, 2.0, 1.2));
//    points_3.push_back(Point_3(3.0, 5.0, 4.5));
//    K::Iso_cuboid_3 c3 = CGAL::bounding_box(points_3.begin(), points_3.end());
//    std::cout << c3 << std::endl;
//
//    Surface_mesh sm;
    std::array<Point, 8> obb_points;
    CGAL::oriented_bounding_box(_pointcloud, obb_points,  CGAL::parameters::use_convex_hull(true).point_map(boxy::Point_map())); //CP::geom_traits(PPoint)
//    std::cout << "Elapsed time: " << timer.time() << std::endl;
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Elapsed time: " << elapsed.count() << " s\n";
    boxy::BBox bbox(obb_points);
//    BBox obb_points;
    return bbox;
}

