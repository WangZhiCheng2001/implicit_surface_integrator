add_requires("mkl_local", {configs = {interface = 32}}) -- to use Eigen with MKL, it has to use LP64 interface instead of ILP64 interface
add_requires("dpl")
add_requires("parallel-hashmap")
add_requires("range-v3")

target("shared_module")
    set_kind("headeronly")
    add_includedirs("./", {public = true})
    add_packages("mkl_local", "dpl", "parallel-hashmap", "range-v3", {public = true})
target_end()