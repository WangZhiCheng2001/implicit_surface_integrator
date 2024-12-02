add_requires("eigen-latest")

internal_library("implicit_surface_network_process", "ISNP", os.scriptdir())
    add_rules("config.indirect_predicates.flags")
    add_deps("implicit_arrangements", "shared_module")
    add_defines("RELEASE_BRANCH")
    add_packages("eigen-latest", {public = true})