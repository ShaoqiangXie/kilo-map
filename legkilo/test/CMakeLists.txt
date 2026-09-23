cmake_minimum_required(VERSION 3.5)

add_executable(test_tbb test_tbb.cc)

target_link_libraries(test_tbb PRIVATE tbb)

add_executable(test_intensity test_intensity.cc)
target_link_libraries(test_intensity PRIVATE legkilo.lib)
foreach(case model jacobian registration frontend)
  add_test(NAME intensity_${case} COMMAND test_intensity ${case})
endforeach()
