set_arch("x64")
set_languages("c11", "c++17")
set_toolchains("clang-cl")

includes("xmake/rules/**/xmake.lua")
add_rules("lsp.msvc_inc_inject")
-- add_rules("plugin.vsxmake.autoupdate")
-- add_cxxflags("/Zc:__cplusplus")
add_rules("mode.debug", "mode.release")

add_repositories("local-repo xmake")
set_runtimes("MD") -- force /MD runtime for MSVC, since MKL only accepts /MD on windows

includes("./*/xmake.lua")