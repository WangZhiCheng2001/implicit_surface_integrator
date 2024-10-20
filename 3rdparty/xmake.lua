target("indirect_predicates")
    set_kind("headeronly")
    add_includedirs("indirect_predicates/include", {public = true})
    add_vectorexts("all")
target_end()

rule("config.indirect_predicates.flags")
    on_config(function (target)
        if (target:has_tool("cxx", "cl", "clang_cl")) then
            target:add("cxflags", "/fp:strict")
            target:add("cxflags", "/Oi")
            target:add("defines", "_CRT_SECURE_NO_WARNINGS")
            target:add("cxflags", "/link /STACK:8421376")
        else
            target:add("cxflags", "-Wl,-z,stacksize=8421376")
            target:add("cxflags", "-frounding-math")
        end
    end)
rule_end()