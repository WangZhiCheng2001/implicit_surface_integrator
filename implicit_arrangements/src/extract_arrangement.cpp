#include "extract_arrangement.hpp"
#include "ia_structure.hpp"
#include "robust_assert.hpp"

Arrangement2D extract_arrangement(IAComplex<2>&& ia_complex)
{
    Arrangement2D ia;
    ia.num_vertices = static_cast<uint32_t>(ia_complex.vertices.size());
    ia.points       = static_cast<Point2D*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(Point2D) * ia.num_vertices));
    memcpy(ia.points, ia_complex.vertices.data(), sizeof(Point2D) * ia.num_vertices);

    auto& edges  = ia_complex.edges;
    ia.num_edges = static_cast<uint32_t>(edges.size());
    ia.vertices  = static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.num_edges * 2));
    ia.supporting_lines =
        static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.num_edges));
    ia.positive_cells = static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.num_edges));
    ia.negative_cells = static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.num_edges));

    for (size_t i = 0; i < ia.num_edges; i++) {
        auto& ce               = edges[i];
        ia.vertices[2 * i]     = ce.vertices[0];
        ia.vertices[2 * i + 1] = ce.vertices[1];
        ia.supporting_lines[i] = ce.supporting_plane;
        ia.positive_cells[i]   = ce.positive_face;
        ia.negative_cells[i]   = ce.negative_face;
    }

    auto& faces  = ia_complex.faces;
    ia.num_faces = static_cast<uint32_t>(faces.size());
    ia.edges     = static_cast<uint32_t**>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t*) * ia.num_faces));
    ia.face_edges_count =
        static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.num_faces));

    for (size_t i = 0; i < ia.num_faces; i++) {
        auto& cf               = faces[i];
        ia.face_edges_count[i] = static_cast<uint32_t>(cf.edges.size());
        ia.edges[i] =
            static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.face_edges_count[i]));
        memcpy(ia.edges[i], cf.edges.data(), sizeof(uint32_t) * ia.face_edges_count[i]);
    }

    return ia;
}

Arrangement3D extract_arrangement(IAComplex<3>&& ia_complex)
{
    Arrangement3D ia;
    ia.num_vertices = static_cast<uint32_t>(ia_complex.vertices.size());
    ia.points       = static_cast<Point3D*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(Point3D) * ia.num_vertices));
    memcpy(ia.points, ia_complex.vertices.data(), sizeof(Point3D) * ia.num_vertices);

    auto& edges  = ia_complex.edges;
    auto& faces  = ia_complex.faces;
    ia.num_faces = static_cast<uint32_t>(faces.size());
    ia.vertices  = static_cast<uint32_t**>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t*) * ia.num_faces));
    ia.face_vertices_count =
        static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.num_faces));
    ia.positive_cells = static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.num_faces));
    ia.negative_cells = static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.num_faces));
    ia.supporting_planes =
        static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.num_faces));

    for (size_t i = 0; i < ia.num_faces; i++) {
        auto& cf                  = faces[i];
        ia.face_vertices_count[i] = static_cast<uint32_t>(cf.edges.size());
        ROBUST_ASSERT(ia.face_vertices_count[i] >= 3);
        ia.vertices[i] = static_cast<uint32_t*>(
            ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.face_vertices_count[i]));

        for (size_t j = 0; j < ia.face_vertices_count[i]; j++) {
            auto& curr_e = edges[cf.edges[j]];
            auto& next_e = edges[cf.edges[(j + 1) % ia.face_vertices_count[i]]];
            if (curr_e.vertices[0] == next_e.vertices[0] || curr_e.vertices[0] == next_e.vertices[1]) {
                ia.vertices[i][j] = curr_e.vertices[0];
            } else {
                ia.vertices[i][j] = curr_e.vertices[1];
            }
        }

        ia.positive_cells[i]    = cf.positive_cell;
        ia.negative_cells[i]    = cf.negative_cell;
        ia.supporting_planes[i] = cf.supporting_plane;
    }

    auto& cells  = ia_complex.cells;
    ia.num_cells = static_cast<uint32_t>(cells.size());
    ia.faces     = static_cast<uint32_t**>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t*) * ia.num_cells));
    ia.cell_faces_count =
        static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.num_cells));

    for (size_t i = 0; i < ia.num_cells; i++) {
        auto& cc               = cells[i];
        ia.cell_faces_count[i] = static_cast<uint32_t>(cc.faces.size());
        ia.faces[i] =
            static_cast<uint32_t*>(ScalableMemoryPoolSingleton::instance().malloc(sizeof(uint32_t) * ia.cell_faces_count[i]));
        memcpy(ia.faces[i], cc.faces.data(), sizeof(uint32_t) * ia.cell_faces_count[i]);
    }

    return ia;
}