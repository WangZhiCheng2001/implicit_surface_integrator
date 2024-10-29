#include <assert.h>

#include <background_mesh.hpp>
#include <algorithm/glue_algorithm.hpp>

struct TetrahedronVertexIndexGroup {
    uint32_t v00, v01, v02, v03;
    uint32_t v10, v11, v12, v13;
    uint32_t v20, v21, v22, v23;
    uint32_t v30, v31, v32, v33;
    uint32_t v40, v41, v42, v43;
};

tetrahedron_mesh_t generate_tetrahedron_background_mesh(uint32_t                             resolution,
                                                        const Eigen::Ref<const raw_point_t>& aabb_min,
                                                        const Eigen::Ref<const raw_point_t>& aabb_max)
{
    assert(resolution > 0);

    const auto         N = resolution + 1;
    tetrahedron_mesh_t mesh{};
    mesh.vertices.resize(N * N * N);
    mesh.indices.resize(resolution * resolution * resolution * 5);

    for (uint32_t i = 0; i < N; ++i) {
        const auto x = (aabb_max[0] - aabb_min[0]) * i / resolution + aabb_min[0];
        for (uint32_t j = 0; j < N; ++j) {
            const auto y = (aabb_max[1] - aabb_min[1]) * j / resolution + aabb_min[1];
            algorithm::for_loop<algorithm::ExecutionPolicySelector::simd_only>(0u, N, [&](uint32_t k) {
                const auto z      = (aabb_max[2] - aabb_min[2]) * k / resolution + aabb_min[2];
                const auto v0     = i * N * N + j * N + k;
                mesh.vertices[v0] = {x, y, z};

                if (i < resolution - 1 && j < resolution - 1 && k < resolution - 1) {
                    size_t     idx = (i * resolution * resolution + j * resolution + k) * 5;
                    const auto v1  = (i + 1) * N * N + j * N + k;
                    const auto v2  = (i + 1) * N * N + (j + 1) * N + k;
                    const auto v3  = i * N * N + (j + 1) * N + k;
                    const auto v4  = i * N * N + j * N + k + 1;
                    const auto v5  = (i + 1) * N * N + j * N + k + 1;
                    const auto v6  = (i + 1) * N * N + (j + 1) * N + k + 1;
                    const auto v7  = i * N * N + (j + 1) * N + k + 1;

                    if ((i + j + k) % 2 == 0)
                        *(TetrahedronVertexIndexGroup*)(mesh.indices.data() + idx) = {v4, v6, v1, v3, v6, v3, v4, v7, v1, v3,
                                                                                      v0, v4, v3, v1, v2, v6, v4, v1, v6, v5};
                    else
                        *(TetrahedronVertexIndexGroup*)(mesh.indices.data() + idx) = {v7, v0, v2, v5, v2, v3, v0, v7, v5, v7,
                                                                                      v0, v4, v7, v2, v6, v5, v0, v1, v2, v5};
                }
            });
        }
    }

    return mesh;
}