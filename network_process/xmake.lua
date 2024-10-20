add_requires("eigen-latest")

target("implicit_surface_network_process")
    set_kind("object")
    add_vectorexts("all")
    add_includedirs("include", {public = true})
    add_files("src/*.cpp")
    add_packages("eigen-latest", {public = true})
target_end()