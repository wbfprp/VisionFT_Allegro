cmake_minimum_required (VERSION 2.6)
project (BHand)

# build type (Debug/Release/RelWithDebInfo/MinSizeRel)
set (CMAKE_BUILD_TYPE Release)

# version number
set (BHand_VERSION_MAJOR 2)
set (BHand_VERSION_MINOR 0)

# configure a header file to pass some of the CMake settings
# to the source code
#configure_file (
#    "${PROJECT_SOURCE_DIR}/BHandConfig.h.in"
#    "${PROJECT_BINARY_DIR}/BHandConfig.h"
#    )

# add the binary tree to the search path for include files
# so that we will find rMathConfig.h
include_directories ("${PROJECT_BINARY_DIR}")

# add definitions
add_definitions (-DUNICODE -D_UNICODE)

# add RoboticsLab common header files
include_directories (/usr/local/include)

# add the library
add_library (BHand SHARED BHand.cpp BHandGain.cpp)

# add the install targets
install (TARGETS BHand DESTINATION lib)
install (
    FILES BHand.h BHandDef.h
    DESTINATION include/BHand)

