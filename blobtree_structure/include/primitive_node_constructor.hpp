#pragma once

#include "extrude/utils.hpp"
#include "globals.hpp"

static auto plain_desc_copy_constructor = [](auto& desc, void* new_desc) {
    using value_type = std::remove_cv_t<std::remove_reference_t<decltype(desc)>>;
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

static auto extrude_polyline_desc_copy_constructor = [](auto& desc, void* new_desc) {
    extrude_polyline_descriptor_t* new_desc_ptr = static_cast<extrude_polyline_descriptor_t*>(new_desc);

    new_desc_ptr->profile_number = desc.profile_number;

    new_desc_ptr->profiles = (polyline_descriptor_t*)malloc(desc.profile_number * sizeof(polyline_descriptor_t));
    for (int i = 0; i < new_desc_ptr->profile_number; i++) {
        new_desc_ptr->profiles[i].point_number = desc.profiles[i].point_number;
        new_desc_ptr->profiles[i].points =
            (raw_vector3d_t*)malloc(new_desc_ptr->profiles[i].point_number * sizeof(raw_vector3d_t));
        std::copy(desc.profiles[i].points,
                  desc.profiles[i].points + new_desc_ptr->profiles[i].point_number,
                  new_desc_ptr->profiles[i].points);

        new_desc_ptr->profiles[i].bulge_number = desc.profiles[i].bulge_number;
        new_desc_ptr->profiles[i].bulges       = (double*)malloc(new_desc_ptr->profiles[i].bulge_number * sizeof(double));
        std::copy(desc.profiles[i].bulges,
                  desc.profiles[i].bulges + new_desc_ptr->profiles[i].bulge_number,
                  new_desc_ptr->profiles[i].bulges);

        new_desc_ptr->profiles[i].reference_normal = desc.profiles[i].reference_normal;
        new_desc_ptr->profiles[i].is_close         = desc.profiles[i].is_close;
    }

    new_desc_ptr->axis.point_number = desc.axis.point_number;
    new_desc_ptr->axis.points       = (raw_vector3d_t*)malloc(new_desc_ptr->axis.point_number * sizeof(raw_vector3d_t));
    std::copy(desc.axis.points, desc.axis.points + new_desc_ptr->axis.point_number, new_desc_ptr->axis.points);

    new_desc_ptr->axis.bulge_number = desc.axis.bulge_number;
    new_desc_ptr->axis.bulges       = (double*)malloc(new_desc_ptr->axis.bulge_number * sizeof(double));
    std::copy(desc.axis.bulges, desc.axis.bulges + new_desc_ptr->axis.bulge_number, new_desc_ptr->axis.bulges);

    new_desc_ptr->axis.reference_normal = desc.axis.reference_normal;
    new_desc_ptr->axis.is_close         = desc.axis.is_close;
};

static auto extrude_arcline_desc_copy_constructor = [](auto& desc, void* new_desc) {
    extrude_arcline_descriptor_t* new_desc_ptr = static_cast<extrude_arcline_descriptor_t*>(new_desc);

    new_desc_ptr->profile_number = desc.profile_number;

    new_desc_ptr->profiles = (polyline_descriptor_t*)malloc(desc.profile_number * sizeof(polyline_descriptor_t));
    for (int i = 0; i < new_desc_ptr->profile_number; i++) {
        new_desc_ptr->profiles[i].point_number = desc.profiles[i].point_number;
        new_desc_ptr->profiles[i].points =
            (raw_vector3d_t*)malloc(new_desc_ptr->profiles[i].point_number * sizeof(raw_vector3d_t));
        std::copy(desc.profiles[i].points,
                  desc.profiles[i].points + new_desc_ptr->profiles[i].point_number,
                  new_desc_ptr->profiles[i].points);

        new_desc_ptr->profiles[i].bulge_number = desc.profiles[i].bulge_number;
        new_desc_ptr->profiles[i].bulges       = (double*)malloc(new_desc_ptr->profiles[i].bulge_number * sizeof(double));
        std::copy(desc.profiles[i].bulges,
                  desc.profiles[i].bulges + new_desc_ptr->profiles[i].bulge_number,
                  new_desc_ptr->profiles[i].bulges);

        new_desc_ptr->profiles[i].reference_normal = desc.profiles[i].reference_normal;
        new_desc_ptr->profiles[i].is_close         = desc.profiles[i].is_close;
    }

    new_desc_ptr->axis = desc.axis;
};

static auto extrude_helixline_desc_copy_constructor = [](auto& desc, void* new_desc) {
    extrude_helixline_descriptor_t* new_desc_ptr = static_cast<extrude_helixline_descriptor_t*>(new_desc);

    new_desc_ptr->profile_number = desc.profile_number;

    new_desc_ptr->profiles = (polyline_descriptor_t*)malloc(desc.profile_number * sizeof(polyline_descriptor_t));
    for (int i = 0; i < new_desc_ptr->profile_number; i++) {
        new_desc_ptr->profiles[i].point_number = desc.profiles[i].point_number;
        new_desc_ptr->profiles[i].points =
            (raw_vector3d_t*)malloc(new_desc_ptr->profiles[i].point_number * sizeof(raw_vector3d_t));
        std::copy(desc.profiles[i].points,
                  desc.profiles[i].points + new_desc_ptr->profiles[i].point_number,
                  new_desc_ptr->profiles[i].points);

        new_desc_ptr->profiles[i].bulge_number = desc.profiles[i].bulge_number;
        new_desc_ptr->profiles[i].bulges       = (double*)malloc(new_desc_ptr->profiles[i].bulge_number * sizeof(double));
        std::copy(desc.profiles[i].bulges,
                  desc.profiles[i].bulges + new_desc_ptr->profiles[i].bulge_number,
                  new_desc_ptr->profiles[i].bulges);

        new_desc_ptr->profiles[i].reference_normal = desc.profiles[i].reference_normal;
        new_desc_ptr->profiles[i].is_close         = desc.profiles[i].is_close;
    }

    new_desc_ptr->axis = desc.axis;
};

static auto extrude_polyline_desc_move_constructor = [](auto& desc, void* new_desc) {
    extrude_polyline_descriptor_t* new_desc_ptr = static_cast<extrude_polyline_descriptor_t*>(new_desc);

    new_desc_ptr->profile_number = std::move(desc.profile_number);

    new_desc_ptr->profiles = (polyline_descriptor_t*)malloc(desc.profile_number * sizeof(polyline_descriptor_t));
    for (int i = 0; i < new_desc_ptr->profile_number; i++) {
        new_desc_ptr->profiles[i].point_number = std::move(desc.profiles[i].point_number);
        new_desc_ptr->profiles[i].points =
            (raw_vector3d_t*)malloc(new_desc_ptr->profiles[i].point_number * sizeof(raw_vector3d_t));
        std::move(desc.profiles[i].points,
                  desc.profiles[i].points + new_desc_ptr->profiles[i].point_number,
                  new_desc_ptr->profiles[i].points);

        new_desc_ptr->profiles[i].bulge_number = std::move(desc.profiles[i].bulge_number);
        new_desc_ptr->profiles[i].bulges       = (double*)malloc(new_desc_ptr->profiles[i].bulge_number * sizeof(double));
        std::move(desc.profiles[i].bulges,
                  desc.profiles[i].bulges + new_desc_ptr->profiles[i].bulge_number,
                  new_desc_ptr->profiles[i].bulges);

        new_desc_ptr->profiles[i].reference_normal = std::move(desc.profiles[i].reference_normal);
        new_desc_ptr->profiles[i].is_close         = std::move(desc.profiles[i].is_close);
    }

    new_desc_ptr->axis.point_number = std::move(desc.axis.point_number);
    new_desc_ptr->axis.points       = (raw_vector3d_t*)malloc(new_desc_ptr->axis.point_number * sizeof(raw_vector3d_t));
    std::move(desc.axis.points, desc.axis.points + new_desc_ptr->axis.point_number, new_desc_ptr->axis.points);

    new_desc_ptr->axis.bulge_number = std::move(desc.axis.bulge_number);
    new_desc_ptr->axis.bulges       = (double*)malloc(new_desc_ptr->axis.bulge_number * sizeof(double));
    std::move(desc.axis.bulges, desc.axis.bulges + new_desc_ptr->axis.bulge_number, new_desc_ptr->axis.bulges);

    new_desc_ptr->axis.reference_normal = std::move(desc.axis.reference_normal);
    new_desc_ptr->axis.is_close         = std::move(desc.axis.is_close);
};

static auto extrude_arcline_desc_move_constructor = [](auto& desc, void* new_desc) {
    extrude_arcline_descriptor_t* new_desc_ptr = static_cast<extrude_arcline_descriptor_t*>(new_desc);

    new_desc_ptr->profile_number = std::move(desc.profile_number);

    new_desc_ptr->profiles = (polyline_descriptor_t*)malloc(desc.profile_number * sizeof(polyline_descriptor_t));
    for (int i = 0; i < new_desc_ptr->profile_number; i++) {
        new_desc_ptr->profiles[i].point_number = std::move(desc.profiles[i].point_number);
        new_desc_ptr->profiles[i].points =
            (raw_vector3d_t*)malloc(new_desc_ptr->profiles[i].point_number * sizeof(raw_vector3d_t));
        std::move(desc.profiles[i].points,
                  desc.profiles[i].points + new_desc_ptr->profiles[i].point_number,
                  new_desc_ptr->profiles[i].points);

        new_desc_ptr->profiles[i].bulge_number = std::move(desc.profiles[i].bulge_number);
        new_desc_ptr->profiles[i].bulges       = (double*)malloc(new_desc_ptr->profiles[i].bulge_number * sizeof(double));
        std::move(desc.profiles[i].bulges,
                  desc.profiles[i].bulges + new_desc_ptr->profiles[i].bulge_number,
                  new_desc_ptr->profiles[i].bulges);

        new_desc_ptr->profiles[i].reference_normal = std::move(desc.profiles[i].reference_normal);
        new_desc_ptr->profiles[i].is_close         = std::move(desc.profiles[i].is_close);
    }

    new_desc_ptr->axis = std::move(desc.axis);
};

static auto extrude_helixline_desc_move_constructor = [](auto& desc, void* new_desc) {
    extrude_helixline_descriptor_t* new_desc_ptr = static_cast<extrude_helixline_descriptor_t*>(new_desc);

    new_desc_ptr->profile_number = std::move(desc.profile_number);

    new_desc_ptr->profiles = (polyline_descriptor_t*)malloc(desc.profile_number * sizeof(polyline_descriptor_t));
    for (int i = 0; i < new_desc_ptr->profile_number; i++) {
        new_desc_ptr->profiles[i].point_number = std::move(desc.profiles[i].point_number);
        new_desc_ptr->profiles[i].points =
            (raw_vector3d_t*)malloc(new_desc_ptr->profiles[i].point_number * sizeof(raw_vector3d_t));
        std::move(desc.profiles[i].points,
                  desc.profiles[i].points + new_desc_ptr->profiles[i].point_number,
                  new_desc_ptr->profiles[i].points);

        new_desc_ptr->profiles[i].bulge_number = std::move(desc.profiles[i].bulge_number);
        new_desc_ptr->profiles[i].bulges       = (double*)malloc(new_desc_ptr->profiles[i].bulge_number * sizeof(double));
        std::move(desc.profiles[i].bulges,
                  desc.profiles[i].bulges + new_desc_ptr->profiles[i].bulge_number,
                  new_desc_ptr->profiles[i].bulges);

        new_desc_ptr->profiles[i].reference_normal = std::move(desc.profiles[i].reference_normal);
        new_desc_ptr->profiles[i].is_close         = std::move(desc.profiles[i].is_close);
    }

    new_desc_ptr->axis = std::move(desc.axis);
};

// ========================================================================================================

static auto plain_aabb_initer = [](auto& desc, aabb_t& aabb) {};

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

static auto extrude_polyline_aabb_initer = [](auto& desc, aabb_t& aabb) {
    PolyLine              axis = to_polyline(desc.axis);
    std::vector<PolyLine> profiles;
    for (int i = 0; i < desc.profile_number; i++) { profiles.push_back(to_polyline(desc.profiles[i])); }
    ExtrudedSolidPolyline body{profiles, axis};
    auto                  _aabb = body.aabb;
    aabb.min.x()                = body.aabb.min.x();
    aabb.min.y()                = body.aabb.min.y();
    aabb.min.z()                = body.aabb.min.z();
};

static auto extrude_arcline_aabb_initer = [](auto& desc, aabb_t& aabb) {
    ArcLine               axis = to_arcline(desc.axis);
    std::vector<PolyLine> profiles;
    for (int i = 0; i < desc.profile_number; i++) { profiles.push_back(to_polyline(desc.profiles[i])); }
    ExtrudedSolidArcLine body{profiles, axis};
    auto                 _aabb = body.aabb;
    aabb.min.x()               = body.aabb.min.x();
    aabb.min.y()               = body.aabb.min.y();
    aabb.min.z()               = body.aabb.min.z();
};

static auto extrude_helixline_aabb_initer = [](auto& desc, aabb_t& aabb) {
    HelixLine             axis = to_helixline(desc.axis);
    std::vector<PolyLine> profiles;
    for (int i = 0; i < desc.profile_number; i++) { profiles.push_back(to_polyline(desc.profiles[i])); }

    ExtrudedSolidHelixLine body{profiles, axis};
    aabb.min.x() = body.aabb.min.x();
    aabb.min.y() = body.aabb.min.y();
    aabb.min.z() = body.aabb.min.z();
};
