add_requires("mkl_local", {configs = {interface = 32}}) -- to use Eigen with MKL, it has to use LP64 interface instead of ILP64 interface
add_requires("dpl")
add_requires("eigen-latest")
add_requires("parallel-hashmap")
add_requires("range-v3")

target("implicit_surface_network_process")
    set_kind("object")
    add_vectorexts("all")
    add_includedirs("include", {public = true})
    add_files("src/*.cpp")
    add_packages("mkl_local", "dpl", "eigen-latest", "parallel-hashmap", "range-v3", {public = true})
target_end()