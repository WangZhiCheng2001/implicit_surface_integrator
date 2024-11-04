#pragma once

#include <cstdint>

// forward declaration
struct plane_group_t;
struct ia_complex_t;

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
uint32_t add_plane(const plane_group_t& repo, ia_complex_t& ia_complex, uint32_t plane_index);