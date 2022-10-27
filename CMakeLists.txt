cmake_minimum_required(VERSION 3.12)

include("./CMakeShared.txt")

set(PROJ_NAME pr_audio_soloud)

project(${PROJ_NAME} CXX)

add_include_dir(glm)
link_external_library(alsoundsystem)
link_external_library(soloud)

link_generic_module_libraries()
register_pragma_module(${PROJ_NAME})

SET(SOLOUD_BACKEND_SDL2 OFF CACHE BOOL "" FORCE)
SET(SOLOUD_BACKEND_MINIAUDIO ON CACHE BOOL "" FORCE)
set(DEPENDENCY_SOLOUD_INCLUDE "${CMAKE_CURRENT_LIST_DIR}/third_party_libs/soloud/include/" CACHE STRING "" FORCE)

add_subdirectory(third_party_libs/soloud/contrib)

add_dependencies(${PROJ_NAME} soloud)

set_target_properties(${PROJ_NAME} PROPERTIES FOLDER modules/audio/soloud)
set_target_properties(soloud PROPERTIES FOLDER modules/audio/soloud)

set_property(GLOBAL PROPERTY PRAGMA_MODULE_SKIP_TARGET_PROPERTY_FOLDER 1)
