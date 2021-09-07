#include "BoxScene.h"
BoxScene::BoxScene() = default;

void BoxScene::set_pointcloud_path(const std::string& _path) {
    _pointcloud_src = Path(_path);
}

std::string BoxScene::get_pointcloud_path() const{
    return _pointcloud_src;
}

void BoxScene::set_objectlist_path(const std::string& _path) {
    _objectlist_src = Path(_path);
}

std::string BoxScene::get_objectlist_path() const{
    return _objectlist_src;
}

bool BoxScene::load_points() {
    auto points_from_file = helpers::read_file<4>(_pointcloud_src);
    spdlog::info("Loading Points from {}", absolute(_pointcloud_src).c_str());
    std::for_each(points_from_file.cbegin(), points_from_file.cend(), [this](auto const &obj) {
        if(obj[0] != "#") {
            auto p = Point(std::stof(obj[0]), std::stof(obj[1]), std::stof(obj[2]));
            auto i = std::stoi(obj[3]);
            _pointcloud.emplace_back(std::forward_as_tuple(p, i));
        }
    });
    // sort everything by dimension type => x => y => z
    std::sort( _pointcloud.begin(), _pointcloud.end(), helpers::xyzc_compare);
    spdlog::info("{} Points have been added to this scene",_pointcloud.size());
    return true;
}

bool BoxScene::load_objects() {
    if(this->_pointcloud.empty()){
        spdlog::warn("There are no points in this scene, therefore no classes can be added. There will be no points assigned to the objects");
    }
    auto objects = helpers::read_file<2>(_objectlist_src);
    _objectlist.reserve(objects.size());
    for(auto const &obj : helpers::range(objects.begin(), objects.end())){
        if(obj[0] != "#") {
            int id = std::stoi(obj[0]);
            auto obj_name = obj[1];
            boxy::pointcloud_xyzc tmp = this->copy_object_points(id);
            auto box_entity = BoxEntity(static_cast<int>(id), obj_name, tmp);
            _objectlist.insert_or_assign(id, box_entity);
        }
    };
    spdlog::info("{} Classes have been added to this scene",_objectlist.size());
    return true;
}

boxy::pointcloud_xyzc& BoxScene::get_pointcloud(){
    return _pointcloud ;//helpers::xyzc_2_numpy(_pointcloud);
}
/// This should maybe throw if no data is loaded yet.
/// TODO change logic somehow.
/// \return
objectlist BoxScene::get_objectlist() {
    return _objectlist;
}

BoxEntity& BoxScene::get_object(int object_id) {
    if (!_objectlist.count(object_id)) {
        throw std::runtime_error("This id is not present in your dataset");
    }
    return _objectlist.at(object_id);
}
pointcloud_xyzc BoxScene::copy_object_points(uint32_t object_id){
    auto tmp_comp =  XYZC {Point(), object_id};
    pointcloud_xyzc tmp;
    std::copy_if (_pointcloud.begin(), _pointcloud.end(), std::back_inserter(tmp), [object_id](auto& point){return std::get<1>(point) == object_id;});
    return tmp;
    return pointcloud_xyzc(std::lower_bound(_pointcloud.begin(), _pointcloud.end(), tmp_comp, helpers::xyzc_objecttype_compare),
                      std::upper_bound(_pointcloud.begin(), _pointcloud.end(), tmp_comp, helpers::xyzc_objecttype_compare));
}

bool BoxScene::populate(pointcloud_xyzc pcs, std::unordered_map<int, std::string> obj_name_map) {
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

BoxScene create_scene(const std::string &point_src, const std::string &class_src) {
    auto self = BoxScene();
    self.set_pointcloud_path(point_src);
    self.load_points();
    self.set_objectlist_path(class_src);
    self.load_objects();
    return self;
}
