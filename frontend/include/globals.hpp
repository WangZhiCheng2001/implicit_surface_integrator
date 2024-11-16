#pragma once

#include <timer/scoped_timer.hpp>

#include "environment.h"
#include "implicit_surface_network_processor.hpp"

/* global singleton */
extern setting_descriptor              g_settings;
extern labelled_timers_manager         g_timers_manager;
extern ImplicitSurfaceNetworkProcessor g_integrator;