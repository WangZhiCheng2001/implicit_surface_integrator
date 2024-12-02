internal_library("implicit_predicates", "IP", os.scriptdir())
    add_rules("config.indirect_predicates.flags")
    add_deps("indirect_predicates", "shared_module")