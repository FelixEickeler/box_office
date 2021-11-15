//
// Created by felix on 12.06.2021.
//
#include "helpers.h"
//#include "pybind_helpers.h"


//auto helpers::split(const std::string &str, const char delim) {
//    std::vector<std::string> parts;
//    std::string part;
//    std::stringstream ss(str);
//    while (std::getline(ss, part, delim)) {
//        parts.push_back(part);
//    }
//    return parts;
//}

bool helpers::xyzc_compare(const XYZC &p1, const XYZC &p2) {
    // sort by class
    if (std::get<1>(p1) < std::get<1>(p2)) return true;
    else if (std::get<1>(p1) > std::get<1>(p2)) return false;

    if(std::get<0>(p1).x() < std::get<0>(p2).x()) return true;
    else if (std::get<0>(p1).x() > std::get<0>(p2).x()) return false;

    if(std::get<0>(p1).y() < std::get<0>(p2).y()) return true;
    else if (std::get<0>(p1).y() > std::get<0>(p2).y()) return false;

    if(std::get<0>(p1).z() < std::get<0>(p2).z()) return true;
    else if (std::get<0>(p1).z() > std::get<0>(p2).z()) return false;

    return false;
}

bool helpers::xyzc_objecttype_compare(const XYZC &xyzc, const XYZC &lookup) {
    return std::get<1>(xyzc) < std::get<1>(lookup);
}

