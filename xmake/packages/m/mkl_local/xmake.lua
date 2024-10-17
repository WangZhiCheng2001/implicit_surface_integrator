package("mkl_local")

    set_homepage("https://software.intel.com/content/www/us/en/develop/tools/oneapi/components/onemkl.html")
    set_description("Intel® oneAPI Math Kernel Library")

    add_configs("threading", {description = "Choose threading modal for mkl.", default = "tbb", type = "string", values = {"tbb", "openmp", "gomp", "seq"}})
    add_configs("interface", {description = "Choose index integer size for the interface.", default = 32, values = {32, 64}})
    -- add_configs("shared_core", {description = "link shared library for core libraries.", default = true, type = "boolean"})
    -- add_configs("shared_thread", {description = "link shared library for threading libraries.", default = false, type = "boolean"})

    on_fetch("fetch")

    on_load(function (package)
        -- Refer to [oneAPI Math Kernel Library Link Line Advisor](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl-link-line-advisor.html)
        -- to get the link option for MKL library.
        local suffix = (package:config("interface") == 32 and "lp64" or "ilp64")
        if package:config("interface") == 64 then
            package:add("defines", "MKL_ILP64")
        end
        package:add("links", package:is_arch("x64", "x86_64") and "mkl_blas95_" .. suffix or "mkl_blas95")
        package:add("links", package:is_arch("x64", "x86_64") and "mkl_lapack95_" .. suffix or "mkl_lapack95")

        if package:has_tool("cc", "gcc", "gxx") then
            local flags = {"-Wl,--start-group"}
            table.insert(flags, package:is_arch("x64", "x86_64") and "-lmkl_intel_" .. suffix or "-lmkl_intel")
            local threading = package:config("threading")
            if threading == "tbb" then
                table.insert(flags, "-lmkl_tbb_thread")
                package:add("deps", "tbb")
            elseif threading == "seq" then
                table.insert(flags, "-lmkl_sequential")
            elseif threading == "openmp" then
                table.insert(flags, "-lmkl_intel_thread")
                table.insert(flags, "-lomp")
            elseif threading == "gomp" then
                table.insert(flags, "-lmkl_gnu_thread")
                table.insert(flags, "-lgomp")
            end
            table.insert(flags, "-lmkl_core")
            table.insert(flags, "-Wl,--end-group")
            package:add("ldflags", table.concat(flags, " "))
        else
            package:add("links", package:is_arch("x64", "x86_64") and "mkl_intel_" .. suffix or "mkl_intel_c")
            local threading = package:config("threading")
            if threading == "tbb" then
                package:add("links", "mkl_tbb_thread")
                package:add("deps", "tbb")
            elseif threading == "seq" then
                package:add("links", "mkl_sequential")
            elseif threading == "openmp" then
                package:add("links", "mkl_intel_thread", "omp")
            elseif threading == "gomp" then
                package:add("links", "mkl_gnu_thread", "gomp")
            end
            package:add("links", "mkl_core")
        end
    end)

    on_install("windows|!arm64", function (package)
        local headerdir = package:resourcedir("headers")
        if package:is_plat("windows") then
            os.trymv(path.join("Library", "lib"), package:installdir())
            os.trymv(path.join(headerdir, "Library", "include"), package:installdir())
        end
    end)

    on_test(function (package)
        assert(package:check_csnippets({test = [[
            void test() {
                double A[6] = {1.0,2.0,1.0,-3.0,4.0,-1.0};
                double B[6] = {1.0,2.0,1.0,-3.0,4.0,-1.0};
                double C[9] = {.5,.5,.5,.5,.5,.5,.5,.5,.5};
                cblas_dgemm(CblasColMajor,CblasNoTrans,CblasTrans,3,3,2,1,A,3,B,3,2,C,3);
            }
        ]]}, {includes = "mkl_cblas.h"}))
    end)
