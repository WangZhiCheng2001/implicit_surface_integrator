#include <bitset>
#include <fstream>
#include <iomanip>

#include <implicit_predicates.h>
#include <nlohmann/json.hpp>
#include <tbb/tick_count.h>

#include "lut.hpp"
#include "common_structure.hpp"
#include "implicit_arrangement.h"

/* POD data of ia_data */
std::vector<Point3D>               ia_vertices{};
std::vector<std::vector<uint32_t>> ia_face_vertices{};
std::vector<uint32_t*>             ia_faces_ptr{};
std::vector<uint32_t>              ia_face_vertices_count{};
std::vector<uint32_t>              ia_supporting_planes{};
std::vector<uint32_t>              ia_positive_cells{};
std::vector<uint32_t>              ia_negative_cells{};
std::vector<std::vector<uint32_t>> ia_cell_faces{};
std::vector<uint32_t*>             ia_cells_ptr{};
std::vector<uint32_t>              ia_cell_faces_count{};
std::vector<uint32_t>              ia_num_vertices{};
std::vector<uint32_t>              ia_num_faces{};
std::vector<uint32_t>              ia_num_cells{};
/* global variables */
std::vector<Arrangement3D>         ia_data{};
std::vector<uint32_t>              ia_indices{};

// inline void from_json(const nlohmann::json& j, Point3D& p)
// {
//     p.i0 = j[0].get<uint32_t>();
//     p.i1 = j[1].get<uint32_t>();
//     p.i2 = j[2].get<uint32_t>();
// }

// inline void parse_ia_into_pods(const nlohmann::json& j)
// {
//     const auto& vertices = j[0].get<std::vector<Point3D>>();
//     ia_num_vertices.emplace_back(vertices.size());
//     ia_vertices.insert(ia_vertices.end(), vertices.begin(), vertices.end());
//     assert(ia_num_vertices.back() > 0);

//     ia_num_faces.emplace_back(j[1].size());
//     for (const auto& face_entry : j[1]) {
//         ia_face_vertices_count.emplace_back(face_entry[0].size());
//         ia_face_vertices.emplace_back(face_entry[0].get<std::vector<uint32_t>>());
//         ia_faces_ptr.emplace_back(ia_face_vertices.back().data());
//         ia_supporting_planes.emplace_back(face_entry[1].get<uint32_t>());
//         ia_positive_cells.emplace_back(face_entry[2].get<uint32_t>());
//         ia_negative_cells.emplace_back(face_entry[3].get<uint32_t>());
//     }
//     assert(ia_num_faces.back() > 0);

//     ia_num_cells.emplace_back(j[2].size());
//     for (const auto& cell_entry : j[2]) {
//         ia_cell_faces_count.emplace_back(cell_entry.size());
//         ia_cell_faces.emplace_back(cell_entry.get<std::vector<uint32_t>>());
//         ia_cells_ptr.emplace_back(ia_cell_faces.back().data());
//     }
//     assert(ia_num_cells.back() > 0);
// }

// EXTERN_C API bool load_lut()
// {
//     if (!ia_data.empty() && !ia_indices.empty()) return true;

//     auto t0 = tbb::tick_count::now();

//     std::ifstream file("ia_lut.msgpack", std::ios::in | std::ios::binary);
//     if (!file.is_open()) {
//         std::cerr << "Error: IA LUT file does not exist!" << std::endl;
//         return false;
//     }
//     std::vector<char> msgpack(std::istreambuf_iterator<char>(file), {});
//     nlohmann::json    json = nlohmann::json::from_msgpack(msgpack);
//     file.close();

//     ia_indices = json["start_index"].get<std::vector<uint32_t>>();
//     /* a tet has at least 4 vertices and 4 faces, and we'll use this assumption to reverse space for POD data */
//     ia_vertices.reserve(json["data"].size() * 4);
//     ia_face_vertices.reserve(json["data"].size() * 4 * 3); // 3 vertices per triangle face
//     ia_faces_ptr.reserve(json["data"].size() * 4);
//     ia_face_vertices_count.reserve(json["data"].size() * 4);
//     ia_supporting_planes.reserve(json["data"].size() * 4);
//     ia_positive_cells.reserve(json["data"].size() * 4);
//     ia_negative_cells.reserve(json["data"].size() * 4);
//     ia_cell_faces.reserve(json["data"].size() * 4);
//     ia_cells_ptr.reserve(json["data"].size());
//     ia_cell_faces_count.reserve(json["data"].size());
//     ia_num_vertices.reserve(json["data"].size());
//     ia_num_faces.reserve(json["data"].size());
//     ia_num_cells.reserve(json["data"].size());

//     // HINT: change of capcity may happen here.
//     uint32_t i = 0;
//     for (const auto& entry : json["data"]) {
//         parse_ia_into_pods(entry);
//         std::cout << "Parsed IA: " << i << std::endl;
//         i++;
//     }

//     ia_data.resize(json["data"].size());
//     uint32_t vertices_offset{}, faces_offset{}, cells_offset{};
//     for (size_t i = 0; i < json["data"].size(); i++) {
//         auto& ia = ia_data[i];

//         ia.num_vertices = ia_num_vertices[i];
//         ia.points       = ia_vertices.data() + vertices_offset;

//         ia.num_faces           = ia_num_faces[i];
//         ia.vertices            = ia_faces_ptr.data() + faces_offset;
//         ia.face_vertices_count = ia_face_vertices_count.data() + faces_offset;
//         ia.supporting_planes   = ia_supporting_planes.data() + faces_offset;
//         ia.positive_cells      = ia_positive_cells.data() + faces_offset;
//         ia.negative_cells      = ia_negative_cells.data() + faces_offset;

//         ia.num_cells        = ia_num_cells[i];
//         ia.faces            = ia_cells_ptr.data() + cells_offset;
//         ia.cell_faces_count = ia_cell_faces_count.data() + cells_offset;

//         ia.num_unique_planes = 0;
//     }

//     auto t1 = tbb::tick_count::now();
//     std::cout << "Loading LUT took " << std::fixed << std::setprecision(10) << (t1 - t0).seconds() << " seconds." <<
//     std::endl; return true;
// }

EXTERN_C API bool load_lut()
{
    if (!ia_data.empty() && !ia_indices.empty()) return true;

    auto t0 = tbb::tick_count::now();

    std::ifstream ia_data_file("ia_data.dat", std::ios::in | std::ios::binary);
    std::ifstream ia_indices_file("ia_indices.dat", std::ios::in | std::ios::binary);
    if (!ia_data_file.is_open() || !ia_indices_file.is_open()) {
        std::cerr << "Error: IA LUT file does not exist!" << std::endl;
        return false;
    }

    size_t curr_blob_size{};
    ia_data_file >> curr_blob_size;
    ia_num_vertices.resize(curr_blob_size);
    ia_num_faces.resize(curr_blob_size);
    ia_num_cells.resize(curr_blob_size);
    ia_data.resize(curr_blob_size);

    ia_data_file >> curr_blob_size;
    ia_vertices.resize(curr_blob_size);
    for (size_t i = 0; i < curr_blob_size; ++i) ia_data_file >> ia_vertices[i].i0 >> ia_vertices[i].i1 >> ia_vertices[i].i2;

    ia_data_file >> curr_blob_size;
    ia_face_vertices.resize(curr_blob_size);
    ia_faces_ptr.resize(curr_blob_size);
    ia_face_vertices_count.resize(curr_blob_size);
    ia_supporting_planes.resize(curr_blob_size);
    ia_positive_cells.resize(curr_blob_size);
    ia_negative_cells.resize(curr_blob_size);
    for (size_t i = 0; i < curr_blob_size; i++) {
        ia_data_file >> ia_face_vertices_count[i];
        ia_face_vertices[i].resize(ia_face_vertices_count[i]);
        for (size_t j = 0; j < ia_face_vertices_count[i]; j++) ia_data_file >> ia_face_vertices[i][j];
        ia_faces_ptr[i] = ia_face_vertices[i].data();
    }
    for (size_t i = 0; i < curr_blob_size; i++) ia_data_file >> ia_supporting_planes[i];
    for (size_t i = 0; i < curr_blob_size; i++) ia_data_file >> ia_positive_cells[i];
    for (size_t i = 0; i < curr_blob_size; i++) ia_data_file >> ia_negative_cells[i];

    ia_data_file >> curr_blob_size;
    ia_cell_faces.resize(curr_blob_size);
    ia_cells_ptr.resize(curr_blob_size);
    ia_cell_faces_count.resize(curr_blob_size);
    for (size_t i = 0; i < curr_blob_size; i++) {
        ia_data_file >> ia_cell_faces_count[i];
        ia_cell_faces[i].resize(ia_cell_faces_count[i]);
        for (size_t j = 0; j < ia_cell_faces_count[i]; j++) ia_data_file >> ia_cell_faces[i][j];
        ia_cells_ptr[i] = ia_cell_faces[i].data();
    }

    for (size_t i = 0; i < ia_data.size(); i++) ia_data_file >> ia_num_vertices[i];
    for (size_t i = 0; i < ia_data.size(); i++) ia_data_file >> ia_num_faces[i];
    for (size_t i = 0; i < ia_data.size(); i++) ia_data_file >> ia_num_cells[i];

    ia_indices_file >> curr_blob_size;
    ia_indices.resize(curr_blob_size);
    for (size_t i = 0; i < curr_blob_size; i++) ia_indices_file >> ia_indices[i];

    ia_data_file.close();
    ia_indices_file.close();

    uint32_t vertices_offset{}, faces_offset{}, cells_offset{};
    for (size_t i = 0; i < ia_data.size(); i++) {
        auto& ia = ia_data[i];

        ia.num_vertices  = ia_num_vertices[i];
        ia.points        = ia_vertices.data() + vertices_offset;
        vertices_offset += ia.num_vertices;

        ia.num_faces            = ia_num_faces[i];
        ia.vertices             = ia_faces_ptr.data() + faces_offset;
        ia.face_vertices_count  = ia_face_vertices_count.data() + faces_offset;
        ia.supporting_planes    = ia_supporting_planes.data() + faces_offset;
        ia.positive_cells       = ia_positive_cells.data() + faces_offset;
        ia.negative_cells       = ia_negative_cells.data() + faces_offset;
        faces_offset           += ia.num_faces;

        ia.num_cells         = ia_num_cells[i];
        ia.faces             = ia_cells_ptr.data() + cells_offset;
        ia.cell_faces_count  = ia_cell_faces_count.data() + cells_offset;
        cells_offset        += ia.num_cells;

        ia.num_unique_planes = 0;
    }

    auto t1 = tbb::tick_count::now();
    std::cout << "Loading LUT took " << std::fixed << std::setprecision(10) << (t1 - t0).seconds() << " seconds." << std::endl;
    return true;
}

EXTERN_C API void lut_print_test()
{
    const auto& ia = ia_data[0];
    std::cout << "num_vertices: " << ia.num_vertices << std::endl;
    std::cout << "num_faces: " << ia.num_faces << std::endl;
    std::cout << "num_cells: " << ia.num_cells << std::endl;
    std::cout << "points: " << std::endl;
    for (size_t i = 0; i < ia.num_vertices; i++) {
        std::cout << ia.points[i].i0 << " " << ia.points[i].i1 << " " << ia.points[i].i2 << std::endl;
    }
    std::cout << "faces: " << std::endl;
    for (size_t i = 0; i < ia.num_faces; i++) {
        std::cout << "face " << i << std::endl;
        for (size_t j = 0; j < ia.face_vertices_count[i]; j++) { std::cout << ia.vertices[i][j] << " "; }
        std::cout << std::endl;
    }
    std::cout << "supporting_planes: " << std::endl;
    for (size_t i = 0; i < ia.num_faces; i++) {
        std::cout << ia.supporting_planes[i] << std::endl;
    }
    std::cout << "positive_cells: " << std::endl;
    for (size_t i = 0; i < ia.num_faces; i++) {
        std::cout << ia.positive_cells[i] << std::endl;
    }
    std::cout << "negative_cells: " << std::endl;
    for (size_t i = 0; i < ia.num_faces; i++) {
        std::cout << ia.negative_cells[i] << std::endl;
    }
    std::cout << "cells: " << std::endl;
    for (size_t i = 0; i < ia.num_cells; i++) {
        std::cout << "cell " << i << std::endl;
        for (size_t j = 0; j < ia.cell_faces_count[i]; j++) { std::cout << ia.faces[i][j] << " "; }
        std::cout << std::endl;
    }
}

uint32_t ia_compute_outer_index(const Plane3D& p0)
{
    // Plane must not intersect tet at vertices.
    if (p0.f0 == 0 || p0.f1 == 0 || p0.f2 == 0 || p0.f3 == 0) return INVALID_INDEX;

    // To reuse the 2 plane lookup table, we assume the second plane is negative
    // on all tet vertices.
    size_t index = 0;
    if (p0.f0 > 0) index |= 1;
    if (p0.f1 > 0) index |= 4;
    if (p0.f2 > 0) index |= 16;
    if (p0.f3 > 0) index |= 64;

    return index;
}

uint32_t ia_compute_outer_index(const Plane3D& p0, const Plane3D& p1)
{
    // Plane must not intersect tet at vertices.
    if (p0.f0 == 0 || p0.f1 == 0 || p0.f2 == 0 || p0.f3 == 0) return INVALID_INDEX;
    if (p1.f0 == 0 || p1.f1 == 0 || p1.f2 == 0 || p1.f3 == 0) return INVALID_INDEX;

    size_t index = 0;
    if (p0.f0 > 0) index |= 1;
    if (p1.f0 > 0) index |= 2;
    if (p0.f1 > 0) index |= 4;
    if (p1.f1 > 0) index |= 8;
    if (p0.f2 > 0) index |= 16;
    if (p1.f2 > 0) index |= 32;
    if (p0.f3 > 0) index |= 64;
    if (p1.f3 > 0) index |= 128;

    return index;
}

uint32_t ia_compute_inner_index(uint32_t outer_index, const Plane3D& p0, const Plane3D& p1)
{
    std::bitset<2> v0 = outer_index & 3;
    std::bitset<2> v1 = (outer_index >> 2) & 3;
    std::bitset<2> v2 = (outer_index >> 4) & 3;
    std::bitset<2> v3 = (outer_index >> 6) & 3;

    size_t                index      = 0;
    size_t                edge_count = 0;
    std::array<double, 2> pp0, pp1;

    auto add_edge = [&](size_t i, size_t j) -> bool {
        pp0          = {(&p0.f0)[i], (&p0.f0)[j]};
        pp1          = {(&p1.f0)[i], (&p1.f0)[j]};
        const auto s = orient1d(pp0.data(), pp1.data());
        if (s == ORIENTATION_ZERO || s == ORIENTATION_INVALID) return false;

        if (s > 0) index |= (1 << edge_count);
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