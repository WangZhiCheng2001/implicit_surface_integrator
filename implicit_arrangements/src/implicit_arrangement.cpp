#include "arrangement_builder.hpp"

EXTERN_C_BEGIN

API Arrangement2DResult compute_arrangement_2d(const Plane2D* planes, const uint32_t num_planes)
{
    return ArrangementBuilder<2>(planes, num_planes).export_arrangement();
}

API Arrangement3DResult compute_arrangement_3d(const Plane3D* planes, const uint32_t num_planes)
{
    return ArrangementBuilder<3>(planes, num_planes).export_arrangement();
}

API void free_arrangement_2d(Arrangement2D* arr)
{
    if (arr->num_vertices > 0) ScalableMemoryPoolSingleton::instance().free(arr->points);

    if (arr->num_edges > 0) {
        ScalableMemoryPoolSingleton::instance().free(arr->vertices);
        ScalableMemoryPoolSingleton::instance().free(arr->supporting_lines);
        ScalableMemoryPoolSingleton::instance().free(arr->positive_cells);
        ScalableMemoryPoolSingleton::instance().free(arr->negative_cells);
    }

    if (arr->num_faces > 0) {
        for (uint32_t i = 0; i < arr->num_faces; i++) ScalableMemoryPoolSingleton::instance().free(arr->edges[i]);
        ScalableMemoryPoolSingleton::instance().free(arr->edges);
        ScalableMemoryPoolSingleton::instance().free(arr->face_edges_count);
    }

    if (arr->num_unique_planes > 0) {
        ScalableMemoryPoolSingleton::instance().free(arr->unique_plane_indices);
        for (uint32_t i = 0; i < arr->num_unique_planes; i++)
            ScalableMemoryPoolSingleton::instance().free(arr->unique_planes[i]);
        ScalableMemoryPoolSingleton::instance().free(arr->unique_planes);
        ScalableMemoryPoolSingleton::instance().free(arr->unique_plane_count);
        ScalableMemoryPoolSingleton::instance().free(arr->unique_plane_orientations);
    }
}

API void free_arrangement_3d(Arrangement3D* arr)
{
    if (arr->num_vertices > 0) ScalableMemoryPoolSingleton::instance().free(arr->points);

    if (arr->num_faces > 0) {
        for (uint32_t i = 0; i < arr->num_faces; i++) ScalableMemoryPoolSingleton::instance().free(arr->vertices[i]);
        ScalableMemoryPoolSingleton::instance().free(arr->vertices);
        ScalableMemoryPoolSingleton::instance().free(arr->face_vertices_count);
        ScalableMemoryPoolSingleton::instance().free(arr->supporting_planes);
        ScalableMemoryPoolSingleton::instance().free(arr->positive_cells);
        ScalableMemoryPoolSingleton::instance().free(arr->negative_cells);
    }

    if (arr->num_cells > 0) {
        for (uint32_t i = 0; i < arr->num_cells; i++) ScalableMemoryPoolSingleton::instance().free(arr->faces[i]);
        ScalableMemoryPoolSingleton::instance().free(arr->faces);
        ScalableMemoryPoolSingleton::instance().free(arr->cell_faces_count);
    }

    if (arr->num_unique_planes > 0) {
        ScalableMemoryPoolSingleton::instance().free(arr->unique_plane_indices);
        for (uint32_t i = 0; i < arr->num_unique_planes; i++)
            ScalableMemoryPoolSingleton::instance().free(arr->unique_planes[i]);
        ScalableMemoryPoolSingleton::instance().free(arr->unique_planes);
        ScalableMemoryPoolSingleton::instance().free(arr->unique_plane_count);
        ScalableMemoryPoolSingleton::instance().free(arr->unique_plane_orientations);
    }
}

EXTERN_C_END