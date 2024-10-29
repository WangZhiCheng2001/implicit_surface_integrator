#pragma once

#include <container/small_vector.hpp>

// forward declaration
struct Plane2D;
struct Plane3D;

namespace detail
{
template <size_t N>
struct deduce_plane_type {
};

template <>
struct deduce_plane_type<2> {
    using type = Plane2D;
};

template <>
struct deduce_plane_type<3> {
    using type = Plane3D;
};
} // namespace detail

template <size_t N>
struct PlaneGroup {
    using PlaneType = typename detail::deduce_plane_type<N>::type;

    PlaneGroup() = default;

    template <typename InputIt>
    PlaneGroup(InputIt first, InputIt last)
    {
        if constexpr (N == 2) {
            planes = small_vector_mp<PlaneType, N + 1>{
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
            };
        } else {
            planes = small_vector_mp<PlaneType, N + 1>{
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
            };
        }

        planes.insert(planes.end(), std::make_move_iterator(first), std::make_move_iterator(last));
    }

    template <typename Container>
    PlaneGroup(Container&& container)
        : PlaneGroup(std::begin(std::forward<Container>(container)), std::end(std::forward<Container>(container)))
    {
    }

    auto get_plane(uint32_t index) const noexcept { return planes[index]; }

    small_vector_mp<PlaneType, N + 1> planes{};
};

using PlaneGroup2D = PlaneGroup<2>;
using PlaneGroup3D = PlaneGroup<3>;