target("implicit_predicates")
    set_kind("shared")
    add_deps("indirect_predicates")
    add_includedirs("include", {public = true})
    add_files("src/**.cpp")

    on_config(function (target)
        if (target:has_tool("cxx", "cl") or target:has_tool("cxx", "clang-cl")) then
            target:add("cxflags", "/fp:strict")
            target:add("cxflags", "/Oi")
            target:add("defines", "_CRT_SECURE_NO_WARNINGS")
            target:add("cxflags", "/link /STACK:8421376")
        else
            target:add("cxflags", "-Wl,-z,stacksize=8421376")
            target:add("cxflags", "-frounding-math")
        end
    end)
target_end()