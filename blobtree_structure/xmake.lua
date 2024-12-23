add_requires("eigen-latest")

internal_library("blobtree_structure", "BS", os.scriptdir())
    add_rules("config.indirect_predicates.flags")
    add_deps("shared_module")
    add_packages("eigen-latest", {public = true})