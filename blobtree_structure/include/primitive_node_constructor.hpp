#pragma once

#include "globals.hpp"

static auto plain_desc_copy_constructor = [](auto& desc, void* new_desc) {
    using value_type       = std::remove_cv_t<std::remove_reference_t<decltype(desc)>>;
    ::memcpy(new_desc, &desc, sizeof(value_type));
};
static auto plain_desc_move_constructor = [](auto& desc, void* new_desc) {
    using value_type       = std::remove_cv_t<std::remove_reference_t<decltype(desc)>>;
    *(value_type*)new_desc = std::move(desc);
};
static auto mesh_desc_copy_constructor = [](auto& desc, void* new_desc) {
    mesh_descriptor_t* new_desc_ptr = static_cast<mesh_descriptor_t*>(new_desc);

    new_desc_ptr->point_number = desc.point_number;
    new_desc_ptr->face_number  = desc.face_number;
    uint32_t indices_size{};
    for (uint32_t i = 0; i < desc.face_number; ++i) indices_size += desc.faces[i].vertex_count;

    new_desc_ptr->points = (raw_vector3d_t*)malloc(desc.point_number * sizeof(raw_vector3d_t));
    std::copy(desc.points, desc.points + desc.point_number, new_desc_ptr->points);
    new_desc_ptr->faces = (polygon_face_descriptor_t*)malloc(desc.face_number * sizeof(polygon_face_descriptor_t));
    std::copy(desc.faces, desc.faces + desc.face_number, new_desc_ptr->faces);
    new_desc_ptr->indices = (uint32_t*)malloc(indices_size * sizeof(uint32_t));
    std::copy(desc.indices, desc.indices + indices_size, new_desc_ptr->indices);
};
static auto extrude_desc_copy_constructor = [](auto& desc, void* new_desc) {
    extrude_descriptor_t* new_desc_ptr = static_cast<extrude_descriptor_t*>(new_desc);

    new_desc_ptr->edges_number = desc.edges_number;
    new_desc_ptr->extusion     = desc.extusion;

    new_desc_ptr->points = (raw_vector3d_t*)malloc((desc.edges_number - 1) * sizeof(raw_vector3d_t));
    std::copy(desc.points, desc.points + desc.edges_number - 1, new_desc_ptr->points);
    new_desc_ptr->bulges = (double*)malloc(desc.edges_number * sizeof(double));
    std::copy(desc.bulges, desc.bulges + desc.edges_number, new_desc_ptr->bulges);
};
static auto mesh_desc_move_constructor = [](auto& desc, void* new_desc) {
    mesh_descriptor_t* new_desc_ptr = static_cast<mesh_descriptor_t*>(new_desc);

    new_desc_ptr->points = (raw_vector3d_t*)malloc(desc.point_number * sizeof(raw_vector3d_t));
    std::move(desc.points, desc.points + desc.point_number, new_desc_ptr->points);
    new_desc_ptr->point_number = std::move(desc.point_number);

    uint32_t indices_size{};
    for (uint32_t i = 0; i < desc.face_number; ++i) indices_size += desc.faces[i].vertex_count;
    new_desc_ptr->indices = (uint32_t*)malloc(indices_size * sizeof(uint32_t));
    std::move(desc.indices, desc.indices + indices_size, new_desc_ptr->indices);

    new_desc_ptr->faces = (polygon_face_descriptor_t*)malloc(desc.face_number * sizeof(polygon_face_descriptor_t));
    std::move(desc.faces, desc.faces + desc.face_number, new_desc_ptr->faces);
    new_desc_ptr->face_number = std::move(desc.face_number);
};
static auto extrude_desc_move_constructor = [](auto& desc, void* new_desc) {
    extrude_descriptor_t* new_desc_ptr = static_cast<extrude_descriptor_t*>(new_desc);

    new_desc_ptr->extusion = std::move(desc.extusion);

    new_desc_ptr->points = (raw_vector3d_t*)malloc((desc.edges_number - 1) * sizeof(raw_vector3d_t));
    std::move(desc.points, desc.points + desc.edges_number - 1, new_desc_ptr->points);
    new_desc_ptr->bulges = (double*)malloc(desc.edges_number * sizeof(double));
    std::move(desc.bulges, desc.bulges + desc.edges_number, new_desc_ptr->bulges);

    new_desc_ptr->edges_number = std::move(desc.edges_number);
};

// ========================================================================================================

static auto plain_aabb_initer  = [](auto& desc, aabb_t& aabb) {};
static auto sphere_aabb_initer = [](auto& desc, aabb_t& aabb) {
    Eigen::Map<const Eigen::Vector3d> center(&desc.center.x);
    aabb = {center.array() - desc.radius, center.array() + desc.radius};
};
static auto cylinder_aabb_initer = [](auto& desc, aabb_t& aabb) {
    // NOTE: A rough AABB bounding box
    Eigen::Map<const Eigen::Vector3d> bottom_center(&desc.bottom_origion.x), offset(&desc.offset.x);
    aabb.extend(bottom_center.array() + desc.radius);
    aabb.extend(bottom_center.array() - desc.radius);
    aabb.extend(bottom_center.array() + offset.array() + desc.radius);
    aabb.extend(bottom_center.array() + offset.array() - desc.radius);
};
static auto cone_aabb_initer = [](auto& desc, aabb_t& aabb) {
    // NOTE: A rough AABB bounding box
    Eigen::Map<const Eigen::Vector3d> top_point(&desc.top_point.x), bottom_point(&desc.bottom_point.x);
    aabb.extend(top_point.array() + desc.radius1);
    aabb.extend(top_point.array() - desc.radius1);
    aabb.extend(bottom_point.array() + desc.radius2);
    aabb.extend(bottom_point.array() - desc.radius2);
};
static auto box_aabb_initer = [](auto& desc, aabb_t& aabb) {
    Eigen::Map<const Eigen::Vector3d> center(&desc.center.x), half_size(&desc.half_size.x);
    aabb = {center - half_size, center + half_size};
};
static auto mesh_aabb_initer = [](auto& desc, aabb_t& aabb) {
    for (int i = 0; i < desc.point_number; i++) //
        aabb.extend(Eigen::Map<const Eigen::Vector3d>(&desc.points[i].x));
};
static auto extrude_aabb_initer = [](auto& desc, aabb_t& aabb) {
    Eigen::Map<const Eigen::Vector3d> e(&desc.extusion.x);
    // NOTE: Currently only straight edges are considered
    for (int i = 0; i < desc.edges_number; i++) {
        Eigen::Map<const Eigen::Vector3d> p(&desc.points[i].x);
        aabb.extend(p);
        aabb.extend(p + e);
    }
};