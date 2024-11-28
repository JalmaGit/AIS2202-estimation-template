find_package(Eigen3 CONFIG REQUIRED)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

file(COPY "${PROJECT_SOURCE_DIR}/data" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

add_subdirectory(src)
add_executable(state_estimation main.cpp)

target_link_libraries(state_estimation PUBLIC Eigen3::Eigen estimation)
target_include_directories(state_estimation PRIVATE ${RAPIDCSV_INCLUDE_DIRS})
