add_requires("eigen-latest")

internal_library("blobtree_structure", "BPE", os.scriptdir())
    add_rules("config.indirect_predicates.flags")
    add_deps("shared_module")
    add_defines("RELEASE_BRANCH")
    add_packages("eigen-latest", {public = true})