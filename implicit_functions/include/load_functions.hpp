#pragma once

#include <string_view>

#include <container/small_vector.hpp>

#include "implicit_function.hpp"

struct load_functions_result_t {
    small_vector_mp<std::unique_ptr<ImplicitFunction>> functions{};
    bool                                               success{};
};

load_functions_result_t load_functions(const std::string_view& filename);