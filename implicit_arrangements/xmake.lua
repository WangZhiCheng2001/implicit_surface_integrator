add_requires("nlohmann_json")

target("implicit_arrangements")
    add_rules("library.shared")
    add_rules("config.indirect_predicates.flags")
    add_deps("implicit_predicates", "shared_module")
    add_defines("RELEASE_BRANCH")
    add_includedirs("./interface", {public = true})
    add_includedirs("./include")
    add_files("./src/*.cpp")
    add_packages("nlohmann_json")
target_end()

-- target("implicit_arrangements.LUT.generator")
--     set_kind("binary")
--     add_defines("LUT_GENERATE")
--     add_rules("config.indirect_predicates.flags")
--     add_deps("implicit_arrangements")
--     add_includedirs("./include", "./interface")
--     add_files("./src/lut_generator.cpp")
--     add_packages("nlohmann_json")
-- target_end()

target("implicit_arrangements.LUT.load_test")
    set_kind("binary")
    add_rules("config.indirect_predicates.flags")
    add_deps("implicit_arrangements")
    add_files("./test_lut/main.cpp")
target_end()