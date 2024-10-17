#include <assert.h>

#include <background_mesh.h>
#include <glue/glue_algorithm.hpp>

struct TetrahedronVertexIndexGroup {
    size_t v00, v01, v02, v03;
    size_t v10, v11, v12, v13;
    size_t v20, v21, v22, v23;
    size_t v30, v31, v32, v33;
    size_t v40, v41, v42, v43;
};

tetrahedron_mesh_t generate_tetrahedron_background_mesh(size_t             resolution,
                                                        const raw_point_t& aabb_min,
                                                        const raw_point_t& aabb_max)
{
    assert(resolution > 0);

    const auto         N = resolution + 1;
    tetrahedron_mesh_t mesh;
    mesh.vertices.resize(N * N * N);
    mesh.indices.resize(resolution * resolution * resolution * 5);

    algorithm::for_loop<algorithm::ExecutionPolicySelector::unseq_only>(size_t{0}, N, [&](size_t i) {
        const auto x = (aabb_max.x() - aabb_min.x()) * i / resolution + aabb_min.x();
        algorithm::for_loop<algorithm::ExecutionPolicySelector::unseq_only>(size_t{0}, N, [&](size_t j) {
            const auto y = (aabb_max.y() - aabb_min.y()) * j / resolution + aabb_min.y();
            algorithm::for_loop<algorithm::ExecutionPolicySelector::unseq_only>(size_t{0}, N, [&](size_t k) {
                const auto z      = (aabb_max.z() - aabb_min.z()) * k / resolution + aabb_min.z();
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
        });
    });

    return mesh;
}