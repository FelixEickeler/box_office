#include "code/Evaluator.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "code/mvbb_algorithm.h"
#include "code/pybind_helpers.h"

namespace py = pybind11;

BBox create_from_list(py::array_t<float> numpy83){
    auto cpp_vec = helpers::numpy_2_points(numpy83);
    std::array<Point, 8> vertices;
    if(cpp_vec.size() != 8){
        throw std::runtime_error("This numpy array must be 8x3; 8 for each vertex and 3 for x,y,z !");
    }
    std::move(cpp_vec.begin(), cpp_vec.begin() + 8, vertices.begin());
    return BBox(vertices);
}

PYBIND11_MODULE(BoxOffice, module_handle){
    module_handle.doc() = "This module provides a set of tools to execute the MVBB Decomposition!";

    py::class_<BBox>(module_handle, "BoundingBox")
            .def(py::init(&create_from_list))
            .def("get_vertices" ,[](const BBox& self){
                return helpers::bbox_2_numpy(self);
            })
            .def("volume", &BBox::volume)
            .def("intersect", [](const BBox& self, MvbbEvaluator& scene){
                return self.intersect<pointcloud_xyzc>(scene.get_pointcloud());
            });

    using _node = mvbb::FitAndSplitNode<pointcloud_xyzc>;
    py::class_<_node>(module_handle, "BoxNode")
            .def_property_readonly("bounding_box", [](_node & self){
                return self.bounding_box;
            })
            .def("is_final", [](const _node& self){
                return self.final;
            })
            .def("get_points",[](const _node& self){
                return helpers::xyzc_2_numpy(self.get_points());
            });

    py::class_<BoxEntity>(module_handle, "BoxEntity")
            .def("get_id", &BoxEntity::get_id)
            .def("get_points", [](const BoxEntity& self){
                return helpers::xyzc_2_numpy(self.get_points());
            })
            .def("decompose", [](BoxEntity& self, int depth, float gain_threshold) {
                return self.decompose(depth, gain_threshold);
            })
            .def("decompose", [](BoxEntity& self, int depth) {
                return self.decompose(depth);
            });

    py::class_<MvbbEvaluator>(module_handle, "BoxScene")
            .def(py::init<>())
            .def("list_objects", &MvbbEvaluator::get_objectlist) //only work if scene is loaded
            .def("get_points", [](MvbbEvaluator& self) {
                return helpers::xyzc_2_numpy(self.get_pointcloud());
            })
            .def("get_object", [](MvbbEvaluator& self, int object_id) {
                return self.get_object(object_id);
            });
    // get_points

    module_handle.def("create_scene", [](const std::string& point_src, const std::string& class_src) {
        return create_mvbbevaluator(point_src, class_src);
    });


//            .def("calculate_mvbb", [](const MvbbEvaluator& self, int object_id, int depth, float gain_threshold=0.99f) {
//                auto obj_points = self.get_object(object_id);
//                mvbb::Algo_MVBB<pointcloud_xyzc> algo;
//                auto box_decomposition = mvbb::decompose3D(obj_points, &algo, depth, gain_threshold);
////                for(auto& node  : box_decomposition.get_finalized(){
////                    node.
////                }
//                return helpers::bbox_2_numpy(self.bounding_box());
//                });

//    py::class_<boxy::BBox>(module_handle, "BoundingBox")
//            .def(py::init<>())
//            .def_property("vertices", &MvbbEvaluator::get_pointcloud_path, )
//            .def("vertices" ,[](const mvbb::FitAndSplitNode<pointcloud_xyzc> self){
//                    std::array<Point, 8> tmp =  self.bounding_box.get_vertices();
//                    return helpers::xyzc_2_numpy(tmp);
//                })
//            .def_property_readonly("image", [](MvbbEvaluator &self) {
//                py::array out = py::cast(self.make_image());
//                return out;
//            })
//                     .def("multiply_two", &SomeClass::multiply_two)

//            .def("function_that_takes_a_while", &SomeClass::function_that_takes_a_while)
//            ;
}
//    handle.def("get_pointcloud", &get_pointcloud);
//    handle.def("get_pointcloud", &get_pointcloud, py::return_value_policy::move);
//            .def_property("pointcloud_path", &MvbbEvaluator::get_pointcloud_path, &MvbbEvaluator::set_pointcloud_path)
//            .def_property("objectlist_path", &MvbbEvaluator::get_objectlist_path, &MvbbEvaluator::set_objectlist_path)
//
