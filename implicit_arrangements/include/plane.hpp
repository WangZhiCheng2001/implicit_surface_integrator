#pragma once

#include <implicit_arrangement.hpp>

struct plane_group_t {
    plane_group_t() = default;

    template <typename InputIt>
    plane_group_t(InputIt first, InputIt last)
    {
        internal_planes = {
            plane_t{1, 0, 0, 0},
            plane_t{0, 1, 0, 0},
            plane_t{0, 0, 1, 0},
            plane_t{0, 0, 0, 1}
        };

        planes.insert(planes.end(), std::make_move_iterator(first), std::make_move_iterator(last));
    }

    template <typename Container>
    plane_group_t(Container&& container)
        : plane_group_t(std::begin(std::forward<Container>(container)), std::end(std::forward<Container>(container)))
    {
    }

    auto get_plane(uint32_t index) const noexcept
    {
        if (index < 4) return internal_planes[index];
        return planes[index - 4];
    }

    std::array<plane_t, 4> internal_planes{};
    stl_vector_mp<plane_t> planes{};
};