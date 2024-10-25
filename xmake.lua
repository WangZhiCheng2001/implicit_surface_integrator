set_arch("x64")
set_languages("c11", "c++17")
set_toolchains("clang-cl")

includes("xmake/rules/**/xmake.lua")
add_rules("lsp.msvc_inc_inject")
add_rules("mode.debug", "mode.release")

add_repositories("local-repo xmake")
set_runtimes("MD") -- force /MD runtime for MSVC, since MKL only accepts /MD on windows

includes("./3rdparty/xmake.lua")
includes("./shared_module/xmake.lua")

includes("./implicit_**/xmake.lua")
includes("./network_process/xmake.lua")
includes("./application/xmake.lua")