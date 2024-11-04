option("shipping_one")
    set_default(false)
option_end()

option("release_branch")
    set_default(false)
    add_defines("RELEASE_BRANCH")
option_end()

rule("library.shared.internal")
    on_load(function (target)
        local api = target:extraconf("rules", "library.shared.internal", "api")
        if has_config("shipping_one") then
            target:set("kind", "object")
            target:add("defines", api.."_API=", {public = true})
        else
            target:set("kind", "shared")
            target:add("defines", api.."_API=IMPORT_API", {interface = true})
            target:add("defines", api.."_API=EXPORT_API", {public = false})
        end
    end)
rule_end()

rule("library.shared.external")
    on_load(function (target)
        target:set("kind", "shared")
        target:add("defines", "API=IMPORT_API", {interface = true})
        target:add("defines", "API=EXPORT_API", {public = false})
    end)
rule_end()

function internal_library(name, api_name)
    target(name)
        set_group("internal_library")
        add_rules("library.shared.internal", {api = api_name})
        add_includedirs("include", {public = false})
        add_includedirs("interface", {public = true})
        add_files("src/*.cpp")
end

function exposed_library(name)
    target(name)
        set_group("exposed_library")
        add_rules("library.shared.external")
        add_includedirs("include", {public = false})
        add_includedirs("interface", {public = true})
        add_files("src/*.cpp")
end