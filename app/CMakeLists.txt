# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

#####################################################################

# 这个是设置了编译C的参数，这里使用C99并开启三级优化
# 类似的设置还有CMAKE_CPP_FLAGS就是设置编译C++的参数
# 更多的参数就要根据需要看文档了
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -std=c99")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
add_definitions(-std=c++11)
#####################################################################

aux_source_directory(./src/main/cpp DIRMAIN)
aux_source_directory(./src/main/cpp/costmap DIRCOSTMAP)
aux_source_directory(./src/main/cpp/planner DIRPLANNER)
aux_source_directory(./src/main/cpp/transform DIRTF)
aux_source_directory(./src/main/cpp/bfl_core DIRBFL)

#设置OpenCv的路径变量
set(pathToOpenCv /opt/OpenCV-android-sdk)
set(pathToProject /home/shydh/AndroidStudioProjects/AGVP)

#配置加载native依赖
#包含掉OpenCv的头文件目录
include_directories(${pathToOpenCv}/sdk/native/jni/include)

#动态方式加载
add_library(opencv-lib
            SHARED
            IMPORTED )

#引入libopencv_java3.so文件
set_target_properties(opencv-lib
                      PROPERTIES
                      IMPORTED_LOCATION
                      #包含libopencv_java3.so这个库
                      ${pathToProject}/app/src/main/jniLibs/${ANDROID_ABI}/libopencv_java3.so)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.

add_library( # Sets the name of the library.
             robo-lib

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             ${DIRMAIN}
             ${DIRCOSTMAP}
             ${DIRPLANNER}
             ${DIRTF}
             ${DIRBFL}
             )

# Searches for a specified prebuilt library and stores the path as a
# variable. Because CMake includes system libraries in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in this
# build script, prebuilt third-party libraries, or system libraries.

target_link_libraries( # Specifies the target library.
                       robo-lib
                       log
                       android
                       -ljnigraphics
                       opencv-lib

                       # Links the target library to the log library
                       # included in the NDK.
                       ${log-lib} )