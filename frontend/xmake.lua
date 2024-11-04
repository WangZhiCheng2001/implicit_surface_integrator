exposed_library("frontend")
    add_rules("config.indirect_predicates.flags")
    add_deps("implicit_surface_network_process", "implicit_functions", "shared_module")
    add_defines("RELEASE_BRANCH")