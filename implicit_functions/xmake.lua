add_requires("eigen-latest")
add_requires("nlohmann_json")

target("implicit_functions")
    set_kind("static")
    add_vectorexts("fma")
    add_vectorexts("neon")
    add_vectorexts("avx")
    add_vectorexts("avx2")
    add_vectorexts("avx512")
    add_vectorexts("sse")
    add_vectorexts("sse2")
    add_vectorexts("sse3")
    add_vectorexts("ssse3")
    add_vectorexts("sse4.2")
    add_defines("SHARED_MODULE=0")
    add_includedirs("include", {public = true})
    add_files("src/*.cpp")
    add_packages("eigen-latest", {public = true})
    add_packages("nlohmann_json")
    add_deps("shared_module")
target_end()