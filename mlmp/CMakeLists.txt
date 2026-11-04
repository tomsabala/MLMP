cmake_minimum_required(VERSION 3.22)
project(mlmp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ompl 1.7 REQUIRED)
# Header-only Boost is used on both targets (boost/range in solver.h, boost/math/constants in
# kinematic_chain.h, boost/foreach.hpp in BiQRRTImpl.cpp) and needs no find_package; only the
# native CLI links a compiled component.
if(NOT EMSCRIPTEN)
    find_package(Boost REQUIRED COMPONENTS program_options)
endif()
find_package(nlohmann_json 3.11 REQUIRED)

if(EMSCRIPTEN)
    # src/main.cpp is deliberately absent: the wasm entry point owns its own I/O and option
    # handling, and boost::program_options is not linked here.
    add_executable(mlmp_wasm
        src/wasm/entry.cpp
        src/planner/src/BiQRRTImpl.cpp
    )
    target_include_directories(mlmp_wasm PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_link_libraries(mlmp_wasm PRIVATE
        ompl::ompl
        nlohmann_json::nlohmann_json
    )
    target_compile_options(mlmp_wasm PRIVATE -O3 -fwasm-exceptions)
    # Boost 1.74 derives from std::unary_function, which libc++ removed in C++17; OMPL's
    # PlannerDataGraph.h drags boost/graph (hence boost/container_hash) into this target.
    target_compile_definitions(mlmp_wasm PRIVATE _LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION)
    # .mjs so Node and the module worker both treat the glue as an ES module.
    set_target_properties(mlmp_wasm PROPERTIES OUTPUT_NAME mlmp SUFFIX .mjs)
    # STACK_SIZE: Emscripten's default has been 64 KB since 3.1.27, and OMPL's graph and A*
    # code recurses past it. FORCE_FILESYSTEM + exported FS is what lets an embind-only module
    # accept a scene over MEMFS. ENVIRONMENT covers the page's Worker and the Node smoke test.
    target_link_options(mlmp_wasm PRIVATE
        -O3
        -fwasm-exceptions
        --bind
        -sMODULARIZE=1
        -sEXPORT_ES6=1
        -sEXPORT_NAME=createMlmpModule
        -sENVIRONMENT=worker,node
        -sALLOW_MEMORY_GROWTH=1
        -sINITIAL_MEMORY=64MB
        -sSTACK_SIZE=8MB
        -sFORCE_FILESYSTEM=1
        -sEXPORTED_RUNTIME_METHODS=FS
    )
else()
    add_executable(mlmp
        src/main.cpp
        src/planner/src/BiQRRTImpl.cpp
    )

    target_include_directories(mlmp PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_link_libraries(mlmp PRIVATE
        ompl::ompl
        Boost::program_options
        nlohmann_json::nlohmann_json
    )
endif()
