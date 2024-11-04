#include "implicit_arrangement.hpp"
#include "integrator.hpp"
#include "statistics.hpp"
#include "environment.h"
#include "execution.h"

EXTERN_C API void update_setting(const setting_descriptor desc)
{
    g_settings = std::move(desc);
    load_lut();
}

EXTERN_C API void update_scene(const char* input_file) { g_integrator.update_scene(input_file); }

EXTERN_C API void update_environment()
{
    g_integrator.update_background_mesh(-Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
}

EXTERN_C API bool execute_solver()
{
    if (!g_integrator.run(g_timers_manager)) return false;
    g_integrator.compute(g_timers_manager);

    return true;
}

EXTERN_C API void clear_statistics()
{
    g_timers_manager.clear();
}

EXTERN_C API void print_statistics()
{
    g_timers_manager.print();
}