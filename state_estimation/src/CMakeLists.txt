set(sources
        "Kalman_filter.cpp"
        "Fusion.cpp"
)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

add_library(estimation ${sources})
target_link_libraries(estimation PUBLIC
        Eigen3::Eigen
)
target_include_directories(estimation PUBLIC "${CMAKE_SOURCE_DIR}/state_estimation/include" ${RAPIDCSV_INCLUDE_DIRS})