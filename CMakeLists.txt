include(${CMAKE_SOURCE_DIR}/cmake/pr_common.cmake)

set(PROJ_NAME pr_audio_soloud)
pr_add_library(${PROJ_NAME} SHARED)

pr_add_dependency(${PROJ_NAME} shared TARGET PUBLIC)
pr_add_dependency(${PROJ_NAME} alsoundsystem TARGET PUBLIC)

pr_add_headers(${PROJ_NAME} "include/")
pr_add_sources(${PROJ_NAME} "src/")

add_subdirectory(third_party_libs/soloud/contrib)
pr_add_dependency(${PROJ_NAME} soloud TARGET PUBLIC)
target_include_directories(${PROJ_NAME} PUBLIC "third_party_libs/soloud/include/")
set_target_properties(soloud PROPERTIES FOLDER modules/audio/soloud)

pr_finalize(${PROJ_NAME} FOLDER "modules/audio/soloud")
