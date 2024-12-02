option("shipping_one")
    set_default(false)
option_end()

option("release_branch")
    set_default(false)
    add_defines("RELEASE_BRANCH")
option_end()

rule("library.targetdir")
    on_load(function (target)
        if has_config("shipping_one") then
            target:set("targetdir", "$(projectdir)/distribute/lib")
        end
    end)
rule_end()

rule("library.distribute.header")
    after_build(function (target)
        local projectdir = target:extraconf("rules", "library.distribute.header", "projectdir")
        local outputdir = path.join(target:targetdir(), "..", "include")
        if has_config("shipping_one") then
            for _, filepath in ipairs(os.files(path.join(projectdir, "interface", "*.h"))) do 
                local file = io.readfile(filepath)

                local remove_unwanted_header1 = string.gsub(file, "#include <macros.h>", "")
                local remove_unwanted_header2 = string.gsub(remove_unwanted_header1, "#include \"macros.h\"", "")
                local replace_extern1 = string.gsub(remove_unwanted_header2, "EXTERN_C_BEGIN", "extern \"C\" {")
                local replace_extern2 = string.gsub(replace_extern1, "EXTERN_C_END", "}")
                local replace_extern3 = string.gsub(replace_extern2, "EXTERN_C", "extern \"C\"")
                local replace_api1 = string.gsub(replace_extern3, "%a+_API", "__declspec(dllimport)")
                local replace_api2 = string.gsub(replace_api1, "API", "__declspec(dllimport)")
                
                io.writefile(path.join(outputdir, path.filename(filepath)), replace_api2)
            end
        end
    end)
rule_end()

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
        if has_config("shipping_one") then
            if is_mode("debug") then
                target:set("basename", "blobtree_integrald")
            else
                target:set("basename", "blobtree_integral")
            end
        end
    end)
rule_end()

function internal_library(name, api_name, project_basedir)
    target(name)
        set_group("internal_library")
        add_rules("library.shared.internal", {api = api_name})
        add_rules("library.targetdir")
        add_rules("library.distribute.header", {projectdir = project_basedir})
        add_includedirs("include", {public = false})
        add_includedirs("interface", {public = true})
        add_files("src/*.cpp")
end

function exposed_library(name, project_basedir)
    target(name)
        set_group("exposed_library")
        add_rules("library.shared.external")
        add_rules("library.targetdir")
        add_rules("library.distribute.header", {projectdir = project_basedir})
        add_includedirs("include", {public = false})
        add_includedirs("interface", {public = true})
        add_files("src/*.cpp")
end