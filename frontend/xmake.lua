exposed_library("frontend", os.scriptdir())
    add_rules("config.indirect_predicates.flags")
    add_rules("library.force.distribute.header", {headers = path.join(os.scriptdir(), "interface", "construct_helper.hpp")})
    add_deps("implicit_surface_network_process", "primitive_process", "shared_module")