if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()

find_package(Geogram REQUIRED)

if(Geogram_FOUND)
    message(STATUS "Geogram found in ${GEOGRAM_INCLUDE_DIR}")
    include_directories(${GEOGRAM_INCLUDE_DIR})

    if(WIN32) # this macro should normally be defined when configuring the geogram, but it appears not in some cases...
        add_compile_definitions(GEOGRAM_USE_BUILTIN_DEPS)
        add_compile_definitions(GEO_DYNAMIC_LIBS) # ref https://github.com/BrunoLevy/geogram/discussions/376
    endif()

    if(WIN32)
        # Force the dynamic CRT in every configuration, undoing the /MD -> /MT rewrite
        # that FindGeogram.cmake performs when it assumes a static geogram
        # (VORPALINE_BUILD_DYNAMIC=FALSE). A static CRT would mismatch geogram's DLLs and
        # corrupt std::string data when exceptions cross the DLL boundary.
        foreach(config DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
            string(REPLACE "/MT" "/MD" CMAKE_CXX_FLAGS_${config} "${CMAKE_CXX_FLAGS_${config}}")
            string(REPLACE "/MT" "/MD" CMAKE_C_FLAGS_${config} "${CMAKE_C_FLAGS_${config}}")
        endforeach()
        # ... but the *configuration-matched* dynamic CRT, not the Release one in all
        # configurations. geogram is configured with VORPALINE_BUILD_DYNAMIC (see its
        # configure.bat), and its Windows toolchain only rewrites /MD -> /MT for the static
        # build, so its libraries follow the Visual Studio default: /MDd in Debug, /MD
        # otherwise. This variable is read when a target is created, and this directory
        # scope is inherited by everything created below it -- including CoMISo, which
        # FetchContent adds from third_party/. Pinning MultiThreadedDLL here gave CoMISo the
        # Release CRT even in Debug builds, so linking CoMISo.lib into Geolio.dll failed with
        #   LNK2038: mismatch detected for '_ITERATOR_DEBUG_LEVEL': value '0' doesn't match value '2'
        #   LNK2038: mismatch detected for 'RuntimeLibrary': value 'MD_DynamicRelease' doesn't
        #            match value 'MDd_DynamicDebug'
        # because src/geolio is a sibling of third_party and therefore keeps the Visual
        # Studio defaults. Every MSVC object linked into one binary has to agree on the CRT,
        # so follow the same per-configuration choice the rest of the build makes.
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    endif()

    if(WIN32) # copy geogram's runtime DLLs next to the executables (build/bin/<config>)
        if(GEOGRAM_LIBRARY)
            # Derive the geogram build directory (e.g. build/Windows) from the
            # located import library instead of relying on GEOGRAM_PATH, which
            # is not set by FindGeogram.cmake and made this copy a no-op.
            get_filename_component(_GEOGRAM_LIB_DIR "${GEOGRAM_LIBRARY}" DIRECTORY)
            get_filename_component(_GEOGRAM_LIB_PARENT "${_GEOGRAM_LIB_DIR}" DIRECTORY)
            get_filename_component(GEOGRAM_BUILD_DIR "${_GEOGRAM_LIB_PARENT}" DIRECTORY)
            # Copy the whole DLL set (geogram.dll, geogram_gfx.dll, glfw.dll,
            # ...) for every configuration that exists in the geogram build.
            foreach(cfg Release Debug RelWithDebInfo MinSizeRel)
                file(GLOB _GEOGRAM_CFG_DLL_FILES "${GEOGRAM_BUILD_DIR}/bin/${cfg}/*.dll")
                if(_GEOGRAM_CFG_DLL_FILES)
                    message(STATUS "Copying geogram ${cfg} dlls -> ${CMAKE_BINARY_DIR}/bin/${cfg}")
                    file(COPY ${_GEOGRAM_CFG_DLL_FILES} DESTINATION "${CMAKE_BINARY_DIR}/bin/${cfg}")
                endif()
            endforeach()
        endif()
    endif()
else()
    message(WARNING "Geogram not found!")
endif()

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()