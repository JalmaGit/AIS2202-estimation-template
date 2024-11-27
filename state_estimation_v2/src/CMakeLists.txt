set(sources
        "Kalman_filter.cpp"
        "Fusion.cpp"
)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

add_library(estimation_v2 ${sources})
target_link_libraries(estimation_v2 PUBLIC
        Eigen3::Eigen
)
target_include_directories(estimation_v2 PUBLIC "${CMAKE_SOURCE_DIR}/state_estimation_v2/include" ${RAPIDCSV_INCLUDE_DIRS})