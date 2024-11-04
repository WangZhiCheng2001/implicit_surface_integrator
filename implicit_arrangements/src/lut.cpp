#include <bitset>
#include <fstream>
#include <iomanip>

#include <nlohmann/json.hpp>
#include <tbb/tick_count.h>

#include "lut.hpp"
#include "implicit_predicates.hpp"

/* global variables */
stl_vector_mp<arrangement_t> ia_data{};
stl_vector_mp<uint32_t>      ia_indices{};

IA_API bool load_lut()
{
    if (!ia_data.empty() && !ia_indices.empty()) return true;

    auto t0 = tbb::tick_count::now();

    std::ifstream fin("ia_lut.msgpack", std::ios::in | std::ios::binary);
    if (!fin) {
        std::cout << "Simplicial arrangement lookup table file not exist!" << std::endl;
        return false;
    }
    stl_vector_mp<char> msgpack(std::istreambuf_iterator<char>(fin), {});
    nlohmann::json      json = nlohmann::json::from_msgpack(msgpack);
    fin.close();

    const auto deserialize_ar = [](const nlohmann::json& entry) {
        arrangement_t ia{};
        ia.vertices = entry[0].get<stl_vector_mp<point_t>>();
        assert(ia.vertices.size() > 0);

        ia.faces.reserve(entry[1].size());
        for (const auto& face_entry : entry[1]) {
            arrangement_t::face_descriptor f{};
            f.vertices         = face_entry[0].get<stl_vector_mp<uint32_t>>();
            f.supporting_plane = face_entry[1].get<uint32_t>();
            f.positive_cell    = face_entry[2].get<uint32_t>();
            f.negative_cell    = face_entry[3].get<uint32_t>();
            ia.faces.push_back(std::move(f));
        }
        assert(ia.faces.size() > 0);

        ia.cells.reserve(entry[2].size());
        for (const auto& cell_entry : entry[2]) {
            arrangement_t::cell_descriptor c{};
            c.faces = cell_entry.get<stl_vector_mp<uint32_t>>();
            ia.cells.push_back(std::move(c));
        }
        assert(ia.cells.size() > 0);

        return ia;
    };

    ia_indices = json["start_index"].get<stl_vector_mp<uint32_t>>();
    ia_data.reserve(json["data"].size());
    for (const auto& entry : json["data"]) { ia_data.emplace_back(deserialize_ar(entry)); }

    auto t1 = tbb::tick_count::now();
    std::cout << "Loading LUT took " << std::fixed << std::setprecision(10) << (t1 - t0).seconds() << " seconds." << std::endl;

    return true;
}

IA_API void lut_print_test()
{
    const auto& ia = ia_data[0];
    std::cout << "num_vertices: " << ia.vertices.size() << std::endl;
    std::cout << "num_faces: " << ia.faces.size() << std::endl;
    std::cout << "num_cells: " << ia.cells.size() << std::endl;
    std::cout << "points: " << std::endl;
    for (const auto& p : ia.vertices)
        std::cout << "(" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
    std::cout << "faces: " << std::endl;
    for (const auto& f : ia.faces) {
        std::cout << "face: " << std::endl;
        for (const auto& v : f.vertices) std::cout << v << " ";
        std::cout << std::endl;
        std::cout << "supporting_plane: " << f.supporting_plane << std::endl;
        std::cout << "positive_cell: " << f.positive_cell << std::endl;
        std::cout << "negative_cell: " << f.negative_cell << std::endl;
    }
    std::cout << "cells: " << std::endl;
    for (const auto& c : ia.cells) {
        std::cout << "cell: " << std::endl;
        for (const auto& f : c.faces) std::cout << f << " ";
        std::cout << std::endl;
    }
}

uint32_t ia_compute_outer_index(const plane_t& p0)
{
    // Plane must not intersect tet at vertices.
    if (p0[0] == 0 || p0[1] == 0 || p0[2] == 0 || p0[3] == 0) return INVALID_INDEX;

    // To reuse the 2 plane lookup table, we assume the second plane is negative
    // on all tet vertices.
    size_t index = 0;
    if (p0[0] > 0) index |= 1;
    if (p0[1] > 0) index |= 4;
    if (p0[2] > 0) index |= 16;
    if (p0[3] > 0) index |= 64;

    return index;
}

uint32_t ia_compute_outer_index(const plane_t& p0, const plane_t& p1)
{
    // Plane must not intersect tet at vertices.
    if (p0[0] == 0 || p0[1] == 0 || p0[2] == 0 || p0[3] == 0) return INVALID_INDEX;
    if (p1[0] == 0 || p1[1] == 0 || p1[2] == 0 || p1[3] == 0) return INVALID_INDEX;

    size_t index = 0;
    if (p0[0] > 0) index |= 1;
    if (p1[0] > 0) index |= 2;
    if (p0[1] > 0) index |= 4;
    if (p1[1] > 0) index |= 8;
    if (p0[2] > 0) index |= 16;
    if (p1[2] > 0) index |= 32;
    if (p0[3] > 0) index |= 64;
    if (p1[3] > 0) index |= 128;

    return index;
}

uint32_t ia_compute_inner_index(uint32_t outer_index, const plane_t& p0, const plane_t& p1)
{
    std::bitset<2> v0 = outer_index & 3;
    std::bitset<2> v1 = (outer_index >> 2) & 3;
    std::bitset<2> v2 = (outer_index >> 4) & 3;
    std::bitset<2> v3 = (outer_index >> 6) & 3;

    size_t                index      = 0;
    size_t                edge_count = 0;
    std::array<double, 2> pp0, pp1;

    auto add_edge = [&](size_t i, size_t j) -> bool {
        pp0          = {p0[i], p0[j]};
        pp1          = {p1[i], p1[j]};
        const auto s = orient1d(pp0.data(), pp1.data());
        if (s == orientation::zero || s == orientation::invalid) return false;

        if (s == orientation::positive) index |= (1 << edge_count);
        edge_count++;
        return true;
    };

    if ((v0 ^ v1).all())
        if (!add_edge(0, 1)) return INVALID_INDEX;
    if ((v0 ^ v2).all())
        if (!add_edge(0, 2)) return INVALID_INDEX;
    if ((v0 ^ v3).all())
        if (!add_edge(0, 3)) return INVALID_INDEX;
    if ((v1 ^ v2).all())
        if (!add_edge(1, 2)) return INVALID_INDEX;
    if ((v1 ^ v3).all())
        if (!add_edge(1, 3)) return INVALID_INDEX;
    if ((v2 ^ v3).all())
        if (!add_edge(2, 3)) return INVALID_INDEX;

    if (edge_count == 4) {
        assert(index != 6 && index != 9); // Impossible cases.
        if (index < 6) {
        } else if (index < 9) {
            index -= 1;                   // Skipping INVALID_INDEX case with index 6.
        } else {
            index -= 2;                   // Skipping INVALID_INDEX case with index 6 and 9.
        }
    }

    return index;
}