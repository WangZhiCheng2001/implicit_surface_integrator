#pragma once

#include <cstdint>

// forward declaration
template <size_t N>
struct PlaneGroup;
template <size_t N>
struct IAComplex;

/**
 * Insert a plane into the existing arrangement complex.
 *
 * @param[in] repo            Plane repository.
 * @param[in,out] ia_complex  Current arrangement complex.
 * @param[in] plane_index     The index of the plane to be inserted.
 *
 * @return The index of an existing plane that is coplanar with the inserted
 *         plane if exists.  Otherwise, return `INVALID_INDEX`.
 */
uint32_t add_plane(const PlaneGroup<2>& repo, IAComplex<2>& ia_complex, uint32_t plane_index);
uint32_t add_plane(const PlaneGroup<3>& repo, IAComplex<3>& ia_complex, uint32_t plane_index);