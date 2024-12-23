#include <iostream>
#include "primitive_descriptor.h"
#include "globals.hpp"
#ifdef _DEBUG
void output_primitive_node(const primitive_node_t& node)
{
    auto output_point = [](const raw_vector3d_t& point) {
        std::cout << "( " << point.x << ", " << point.y << ", " << point.z << " )" << std::endl;
    };

    auto type = node.type;
    switch (type) {
        case PRIMITIVE_TYPE_CONSTANT: {
            auto desc = static_cast<constant_descriptor_t*>(node.desc);
            std::cout << "constant:" << std::endl;
            std::cout << "\tvalue: " << desc->value << std::endl << std::endl;
            break;
        }
        case PRIMITIVE_TYPE_PLANE: {
            auto desc = static_cast<plane_descriptor_t*>(node.desc);
            std::cout << "plane:" << std::endl;
            std::cout << "\tbase point: ";
            output_point(desc->point);
            std::cout << "\tnormal: ";
            output_point(desc->normal);
            std::cout << std::endl;
            break;
        }
        case PRIMITIVE_TYPE_SPHERE: {
            auto desc = static_cast<sphere_descriptor_t*>(node.desc);
            std::cout << "sphere:" << std::endl;
            std::cout << "\tcenter: ";
            output_point(desc->center);
            std::cout << "\tradius: " << desc->radius << std::endl << std::endl;
            break;
        }
        case PRIMITIVE_TYPE_CYLINDER: {
            auto desc = static_cast<cylinder_descriptor_t*>(node.desc);
            std::cout << "cylinder:" << std::endl;
            std::cout << "\tbottom point: ";
            output_point(desc->bottom_origion);
            std::cout << "\tradius: " << desc->radius << std::endl << std::endl;
            std::cout << "\toffset: ";
            output_point(desc->offset);
            break;
        }
        case PRIMITIVE_TYPE_CONE: {
            auto desc = static_cast<cone_descriptor_t*>(node.desc);
            std::cout << "cone:" << std::endl;
            std::cout << "\tbottom point: ";
            output_point(desc->bottom_point);
            std::cout << "\ttop point: ";
            output_point(desc->top_point);
            std::cout << "\tradius1: " << desc->radius1 << std::endl;
            std::cout << "\tradius2: " << desc->radius2 << std::endl << std::endl;
            break;
        }
        case PRIMITIVE_TYPE_BOX: {
            auto desc = static_cast<box_descriptor_t*>(node.desc);
            std::cout << "box:" << std::endl;
            std::cout << "\tcenter: ";
            output_point(desc->center);
            std::cout << "\thalf_size ";
            output_point(desc->half_size);
            break;
        }
        case PRIMITIVE_TYPE_MESH: {
            auto desc = static_cast<mesh_descriptor_t*>(node.desc);
            std::cout << "mesh:" << std::endl;
            std::cout << "\tpoint number: " << desc->point_number << std::endl;
            for (int i = 0; i < desc->point_number; i++) {
                std::cout << "\t\t( " << desc->points[i].x << ", " << desc->points[i].y << ", " << desc->points[i].z << " )"
                          << std::endl;
            }

            std::cout << "\tfaces number: " << desc->face_number << std::endl;
            for (int i = 0; i < desc->face_number; i++) {
                auto begin  = desc->faces[i].begin_index;
                auto length = desc->faces[i].vertex_count;
                std::cout << "\t\t<" << begin << ", " << length << "> : ";
                for (int j = begin; j < begin + length; j++) { std::cout << desc->indices[j] << " "; }
                std::cout << std::endl;
            }
            break;
        }
        // TODO : add extrude body output
        default: {
            break;
        }
    }
}

#include <map>
void output_blobtree(virtual_node_t node)
{
    std::map<int, std::string> index;
    index[0] = "constant";
    index[1] = "plane";
    index[2] = "sphere";
    index[3] = "cylinder";
    index[4] = "cone";
    index[5] = "box";
    index[6] = "mesh";
    index[7] = "extrude";

    auto               root = structures[node.main_index].nodes[node.inner_index];
    std::queue<node_t> now, next;
    now.push(root);

    std::vector<primitive_node_t> temp;

    while (!now.empty()) {
        auto begin = now.front();
        now.pop();

        if (node_fetch_is_primitive(begin)) {
            std::cout << index[primitives[node_fetch_primitive_index(begin)].type] << "\t\t";
            temp.push_back(primitives[node_fetch_primitive_index(begin)]);
        } else {
            auto op = node_fetch_operation(begin);
            if (op == eNodeOperation::unionOp) {
                std::cout << "or"
                          << "\t\t";
            } else if (op == eNodeOperation::intersectionOp) {
                std::cout << "and"
                          << "\t\t";
            } else if (op == eNodeOperation::differenceOp) {
                std::cout << "sub"
                          << "\t\t";
            }
        }

        if (!node_is_left_child_null(begin)) {
            next.push(structures[node.main_index].nodes[node_fetch_left_child_index(begin)]);
        }
        if (!node_is_right_child_null(begin)) {
            next.push(structures[node.main_index].nodes[node_fetch_right_child_index(begin)]);
        }

        if (now.empty()) {
            now = next;
            while (!next.empty()) { next.pop(); }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;

    for (int i = 0; i < temp.size(); i++) { output_primitive_node(temp[i]); }
}
#endif // _DEBUG