#pragma once

#include <string>

#include <container/small_vector.hpp>

#include "implicit_functions.hpp"

struct load_functions_result_t {
    small_vector_mp<implicit_function_t<double, 3>> functions{};
    bool                                            success{};
};

load_functions_result_t load_functions(const std::string& filename);