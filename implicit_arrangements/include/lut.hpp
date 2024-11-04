#pragma once

#include <implicit_arrangement.hpp>

// Lookup table for simplicial arrangement
extern stl_vector_mp<arrangement_t> ia_data;
extern stl_vector_mp<uint32_t>      ia_indices;

// For 1 plane arrangement lookup.
uint32_t ia_compute_outer_index(const plane_t& p0);

// For 2 plane arrangement lookup.
uint32_t ia_compute_outer_index(const plane_t& p0, const plane_t& p1);
uint32_t ia_compute_inner_index(uint32_t outer_index, const plane_t& p0, const plane_t& p1);