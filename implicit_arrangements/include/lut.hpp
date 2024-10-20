#pragma once

#include <vector>

#include <implicit_arrangement.h>

// Lookup table for simplicial arrangement
extern std::vector<Arrangement3D>         ia_data;
extern std::vector<uint32_t>              ia_indices;

// For 1 plane arrangement lookup.
uint32_t ia_compute_outer_index(const Plane3D& p0);

// For 2 plane arrangement lookup.
uint32_t ia_compute_outer_index(const Plane3D& p0, const Plane3D& p1);
uint32_t ia_compute_inner_index(uint32_t outer_index, const Plane3D& p0, const Plane3D& p1);