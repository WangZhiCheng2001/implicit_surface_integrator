target("indirect_predicates")
    set_kind("headeronly")
    add_includedirs("indirect_predicates/include", {public = true})
    add_vectorexts("all")

    -- on_config(function (target)
    --     if (target:has_tool("cxx", "cl") or target:has_tool("cxx", "clang-cl")) then
    --         target:add("cxflags", "/fp:strict")
    --         target:add("cxflags", "/Oi")
    --         target:add("defines", "_CRT_SECURE_NO_WARNINGS")
    --     else
    --         target:add("cxflags", "-O2")
    --         target:add("")
    --     end
    -- end)
target_end()