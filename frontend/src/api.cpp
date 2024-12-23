#include "implicit_arrangement.hpp"
#include "globals.hpp"

#include "environment.h"
#include "execution.h"

setting_descriptor g_settings{};

EXTERN_C API void update_setting(const setting_descriptor desc)
{
    g_settings = std::move(desc);

    // safety checks
    if (g_settings.resolution <= 0) g_settings.resolution = 99;
    if (g_settings.scene_aabb_margin <= 0) g_settings.scene_aabb_margin = 1e-5;

    load_lut();
}

EXTERN_C API void update_environment(const virtual_node_t* tree_node) { g_processor.preinit(*tree_node); }

EXTERN_C API solve_result_t execute_solver(const virtual_node_t* tree_node) { return g_processor.run(*tree_node); }

EXTERN_C API void clear_solver_cache() { g_processor.clear(); }

EXTERN_C API void clear_statistics() { g_timers_manager.clear(); }

EXTERN_C API void print_statistics() { g_timers_manager.print(); }