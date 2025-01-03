target("indirect_predicates")
    set_kind("headeronly")
    add_includedirs("indirect_predicates/include", {public = true})
    add_vectorexts("fma")
    add_vectorexts("neon")
    add_vectorexts("avx")
    add_vectorexts("avx2")
    add_vectorexts("sse")
    add_vectorexts("sse2")
    add_vectorexts("sse3")
    add_vectorexts("ssse3")
    add_vectorexts("sse4.2")
target_end()

rule("config.indirect_predicates.flags")
    on_config(function (target)
        target:set("fpmodels", "strict")
        if (target:has_tool("cxx", "cl", "clang_cl")) then
            target:add("cxflags", "/Oi")
            target:add("defines", "_CRT_SECURE_NO_WARNINGS")
            -- target:add("cxflags", "/link /STACK:8421376")
        else
            -- target:add("cxflags", "-Wl,-z,stacksize=8421376")
        end
        target:add("vectorexts", "fma")
        target:add("vectorexts", "neon")
        target:add("vectorexts", "avx")
        target:add("vectorexts", "avx2")
        target:add("vectorexts", "sse")
        target:add("vectorexts", "sse2")
        target:add("vectorexts", "sse3")
        target:add("vectorexts", "ssse3")
        target:add("vectorexts", "sse4.2")
    end)
rule_end()