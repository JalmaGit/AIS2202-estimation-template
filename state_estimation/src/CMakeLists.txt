set(sources
        "Kalman_filter.cpp"
        "Fusion.cpp"
)

add_library(estimation ${sources})
target_link_libraries(estimation PUBLIC
        Eigen3::Eigen
)
target_include_directories(estimation PUBLIC "${CMAKE_SOURCE_DIR}/state_estimation/include")