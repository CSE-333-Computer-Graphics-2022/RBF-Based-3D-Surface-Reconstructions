# ------------------------------------------------------------------------
# include directory
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ------------------------------------------------------------------------
# Keyframe selection app
SET(APP_KEYFRAMES AppKeyframes)
ADD_EXECUTABLE(${APP_KEYFRAMES}
    include/nv/app_keyframes.h
    src/app_keyframes.cpp
)
TARGET_LINK_LIBRARIES(${APP_KEYFRAMES} ${Intrinsic3D_LIBRARIES})
TARGET_COMPILE_OPTIONS(${APP_KEYFRAMES} PRIVATE -std=c++11)

# ------------------------------------------------------------------------
# Fusion app
SET(APP_FUSION AppFusion)
ADD_EXECUTABLE(${APP_FUSION}
    include/nv/app_fusion.h
    src/app_fusion.cpp
)
TARGET_LINK_LIBRARIES(${APP_FUSION} ${Intrinsic3D_LIBRARIES})
TARGET_COMPILE_OPTIONS(${APP_FUSION} PRIVATE -std=c++11)

# ------------------------------------------------------------------------
# Intrinsic3D app
SET(APP_INTRINSIC3D AppIntrinsic3D)
ADD_EXECUTABLE(${APP_INTRINSIC3D}
    include/nv/app_intrinsic3d.h
    src/app_intrinsic3d.cpp
)
TARGET_LINK_LIBRARIES(${APP_INTRINSIC3D} ${Intrinsic3D_LIBRARIES})
TARGET_COMPILE_OPTIONS(${APP_INTRINSIC3D} PRIVATE -std=c++11)
