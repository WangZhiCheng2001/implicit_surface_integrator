#include "arrangement_builder.hpp"

IA_API arrangement_t compute_arrangement(const stl_vector_mp<plane_t>& planes)
{
    return arrangement_builder(planes).export_arrangement();
}