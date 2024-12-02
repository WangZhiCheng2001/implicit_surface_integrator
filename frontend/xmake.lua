exposed_library("frontend", os.scriptdir())
    add_rules("config.indirect_predicates.flags")
    add_deps("implicit_surface_network_process", "blobtree_structure", "shared_module")
    add_defines("RELEASE_BRANCH")