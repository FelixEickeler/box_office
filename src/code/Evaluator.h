#ifndef MVBB_EVALUATOR_H
#define MVBB_EVALUATOR_H
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
//#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>
//#include <pybind11/numpy.h>
#include <CGAL/Optimal_bounding_box/Oriented_bounding_box_traits_3.h>

#include <filesystem>
#include <string>
#include "typedefs.h"
#include "BoxEntity.h"


//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Surface_mesh.h>
#include <CGAL/bounding_box.h>
#include "BoxEntity.h"

//#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
//#include <CGAL/Polygon_mesh_processing/measure.h>
//#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

//using Path = std::filesystem::path;
using objectlist =  std::unordered_map<int, BoxEntity>;

class MvbbEvaluator{
    boxy::Path _pointcloud_src = "";
    boxy::Path _objectlist_src = "";
    boxy::pointcloud_xyzc _pointcloud;
    objectlist _objectlist;
    std::string hash_key;

    public:
        MvbbEvaluator();
        void set_pointcloud_path(const std::string& _path);
        std::string get_pointcloud_path() const;
        void set_objectlist_path(const std::string& _path);
        std::string get_objectlist_path() const;
        bool load_points();
        bool load_objects();

        boxy::BBox bounding_box() const;
        boxy::pointcloud_xyzc& get_pointcloud();
        objectlist get_objectlist();
        BoxEntity& get_object(uint32_t object_id);
        pointcloud_xyzc copy_object_points(uint32_t object_id);
};

MvbbEvaluator create_mvbbevaluator(const std::string& point_src, const std::string& class_src);


#endif //MVBB_EVALUATOR_H