#ifndef BOX_SCENE_H
#define BOX_SCENE_H

#include <CGAL/Optimal_bounding_box/Oriented_bounding_box_traits_3.h>
#include <filesystem>
#include <string>
#include "typedefs.h"
#include "BoxEntity.h"
#include <CGAL/bounding_box.h>
#include "BoxEntity.h"

using objectlist =  std::unordered_map<int, BoxEntity>;

class BoxScene{
    boxy::Path _pointcloud_src = "";
    boxy::Path _objectlist_src = "";
    boxy::pointcloud_xyzc _pointcloud;
    objectlist _objectlist;
    std::string hash_key;

    public:
        BoxScene();
        void set_pointcloud_path(const std::string& _path);
        std::string get_pointcloud_path() const;
        void set_objectlist_path(const std::string& _path);
        std::string get_objectlist_path() const;
        bool load_points();
        bool load_objects();
        bool populate(pointcloud_xyzc pcs, std::unordered_map<int, std::string> obj_name_map);

        boxy::pointcloud_xyzc& get_pointcloud();
        objectlist get_objectlist();
        BoxEntity& get_object(int object_id);
        pointcloud_xyzc copy_object_points(uint32_t object_id);
};

BoxScene create_scene(const std::string& point_src, const std::string& class_src);


#endif //BOX_SCENE_H