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

API void free_arrangement_2d(Arrangement2DResult* arr)
{
    if (!arr->is_runtime_computed) return;

    if (arr->arrangement.num_vertices > 0) ScalableMemoryPoolSingleton::instance().free(arr->arrangement.points);

    if (arr->arrangement.num_edges > 0) {
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.vertices);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.supporting_lines);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.positive_cells);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.negative_cells);
    }

    if (arr->arrangement.num_faces > 0) {
        for (uint32_t i = 0; i < arr->arrangement.num_faces; i++)
            ScalableMemoryPoolSingleton::instance().free(arr->arrangement.edges[i]);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.edges);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.face_edges_count);
    }

    if (arr->arrangement.num_unique_planes > 0) {
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.unique_plane_indices);
        for (uint32_t i = 0; i < arr->arrangement.num_unique_planes; i++)
            ScalableMemoryPoolSingleton::instance().free(arr->arrangement.unique_planes[i]);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.unique_planes);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.unique_plane_count);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.unique_plane_orientations);
    }
}

API void free_arrangement_3d(Arrangement3DResult* arr)
{
    if (!arr->is_runtime_computed) return;

    if (arr->arrangement.num_vertices > 0) ScalableMemoryPoolSingleton::instance().free(arr->arrangement.points);

    if (arr->arrangement.num_faces > 0) {
        for (uint32_t i = 0; i < arr->arrangement.num_faces; i++)
            ScalableMemoryPoolSingleton::instance().free(arr->arrangement.vertices[i]);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.vertices);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.edge_vertices_count);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.supporting_planes);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.positive_cells);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.negative_cells);
    }

    if (arr->arrangement.num_cells > 0) {
        for (uint32_t i = 0; i < arr->arrangement.num_cells; i++)
            ScalableMemoryPoolSingleton::instance().free(arr->arrangement.faces[i]);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.faces);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.cell_faces_count);
    }

    if (arr->arrangement.num_unique_planes > 0) {
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.unique_plane_indices);
        for (uint32_t i = 0; i < arr->arrangement.num_unique_planes; i++)
            ScalableMemoryPoolSingleton::instance().free(arr->arrangement.unique_planes[i]);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.unique_planes);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.unique_plane_count);
        ScalableMemoryPoolSingleton::instance().free(arr->arrangement.unique_plane_orientations);
    }
}

EXTERN_C_END