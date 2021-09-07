#include "code/BoxScene.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "code/mvbb_algorithms.h"
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

BoxScene create_scene_from_2numpy(py::array_t<float> nx4, std::unordered_map<int, std::string> object_name_map){
    auto pointcloud = helpers::numpy_2_xyzc(nx4);
//    auto obj_list = helpers::numpy_2_idclass
    BoxScene scene;
    scene.populate(pointcloud, object_name_map);
    return scene;
}

PYBIND11_MODULE(BoxOffice, module_handle) {
    module_handle.doc() = "This module provides a set of tools to execute the MVBB Decomposition!";

    py::class_<BBox>(module_handle, "BoundingBox")
            .def(py::init(&create_from_list))
            .def("get_vertices", [](const BBox &self) {
                return helpers::bbox_2_numpy(self);
            })
            .def("volume", &BBox::volume)
            .def("intersect", [](const BBox &self, BoxScene &scene) {
                return self.intersect<pointcloud_xyzc>(scene.get_pointcloud());
            });

    using _node = FitAndSplitNode<pointcloud_xyzc>;
    py::class_<_node>(module_handle, "BoxNode")
            .def_property_readonly("bounding_box", [](_node &self) {
                return self.bounding_box;
            })
            .def("is_final", [](const _node &self) {
                return self.final;
            })
            .def("get_points", [](const _node &self) {
                return helpers::xyzc_2_numpy(self.get_points());
            });

    py::class_<BoxEntity>(module_handle, "BoxEntity")
            .def("get_id", &BoxEntity::get_id)
            .def("get_points", [](const BoxEntity &self) {
                return helpers::xyzc_2_numpy(self.get_points());
            })
            .def("get_name", &BoxEntity::get_name)
            .def("decompose", [](BoxEntity &self, int depth, float gain_threshold) {
                return self.decompose(depth, gain_threshold);
            })
            .def("decompose", [](BoxEntity &self, int depth) {
                return self.decompose(depth);
            })
            .def(py::pickle([](const BoxEntity &self) {
                return py::make_tuple(self.get_id(), self.get_name(), helpers::xyzc_2_numpy(self.get_points()));
            }, [](py::tuple t) {
                if (t.size() != 3)
                    throw std::runtime_error("Invalid state!");
                auto _pcs = t[2].cast<py::array>();
                // TODO add checks here
                BoxEntity p(t[0].cast<uint32_t>(), t[1].cast<std::string>(), helpers::numpy_2_xyzc(_pcs));
                return p;
            }));

    py::class_<BoxScene>(module_handle, "BoxScene")
            .def(py::init<>())
            .def("list_objects", &BoxScene::get_objectlist) //only work if scene is loaded
            .def("get_points", [](BoxScene &self) {
                return helpers::xyzc_2_numpy(self.get_pointcloud());
            })
            .def("get_object", [](BoxScene &self, int object_id) {
                return self.get_object(object_id);
            });
    // get_points

    module_handle.def("create_scene", [](const std::string &point_src, const std::string &class_src) {
        return create_scene(point_src, class_src);
    });

    module_handle.def("set_logger_level", [](const std::string &log_level){
       if (log_level == "trace"){ spdlog::set_level(spdlog::level::trace); return;}
       if (log_level == "debug"){ spdlog::set_level(spdlog::level::trace); return;}
       if (log_level == "info"){ spdlog::set_level(spdlog::level::trace); return;}
       if (log_level == "warn"){ spdlog::set_level(spdlog::level::trace); return;}
       if (log_level == "err"){ spdlog::set_level(spdlog::level::trace); return;}
       if (log_level == "critical"){ spdlog::set_level(spdlog::level::trace); return;}
       if (log_level == "off"){ spdlog::set_level(spdlog::level::trace); return;}
       spdlog::warn("Logger Level {} does not exist. Use: trace, debug, info, warn, err, critical or off", log_level);
    });

    module_handle.def("create_scene", [](const py::array &point_src, std::unordered_map<int, std::string> class_src) {
        return create_scene_from_2numpy(point_src, std::move(class_src));
    });

}