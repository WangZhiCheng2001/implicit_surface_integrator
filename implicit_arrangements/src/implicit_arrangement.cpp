#include "arrangement_builder.hpp"

IA_API arrangement_t compute_arrangement(const tbb_vector_mp<plane_t>& planes)
{
    return arrangement_builder(planes).export_arrangement();
}