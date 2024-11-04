#include "extract_arrangement.hpp"
#include "ia_structure.hpp"
#include "robust_assert.hpp"

arrangement_t extract_arrangement(ia_complex_t&& ia_complex)
{
    arrangement_t ia{};
    ia.vertices = std::move(ia_complex.vertices);

    auto&  edges     = ia_complex.edges;
    auto&  faces     = ia_complex.faces;
    size_t num_faces = faces.size();
    ia.faces.resize(num_faces);

    for (size_t i = 0; i < num_faces; i++) {
        auto&        cf           = faces[i];
        auto&        f            = ia.faces[i];
        const size_t num_bd_edges = cf.edges.size();
        ROBUST_ASSERT(num_bd_edges >= 3);
        f.vertices.reserve(num_bd_edges);

        for (size_t j = 0; j < num_bd_edges; j++) {
            auto& curr_e = edges[cf.edges[j]];
            auto& next_e = edges[cf.edges[(j + 1) % num_bd_edges]];
            if (curr_e.vertices[0] == next_e.vertices[0] || curr_e.vertices[0] == next_e.vertices[1]) {
                f.vertices.emplace_back(curr_e.vertices[0]);
            } else {
                ROBUST_ASSERT(curr_e.vertices[1] == next_e.vertices[0] || curr_e.vertices[1] == next_e.vertices[1]);
                f.vertices.emplace_back(curr_e.vertices[1]);
            }
        }

        f.positive_cell    = cf.positive_cell;
        f.negative_cell    = cf.negative_cell;
        f.supporting_plane = cf.supporting_plane;
    }

    auto&  cells     = ia_complex.cells;
    size_t num_cells = cells.size();
    ia.cells.resize(num_cells);

    for (size_t i = 0; i < num_cells; i++) {
        auto& cc = cells[i];
        auto& c  = ia.cells[i];
        c.faces  = std::move(cc.faces);
    }

    return ia;
}