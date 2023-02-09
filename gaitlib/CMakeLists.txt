cmake_minimum_required(VERSION 3.9)
project(gaitlib)

# create the turtlelib library
add_library(gaitlib src/gaits.cpp)
# The add_library function just added turtlelib as a "target"
# A "target" is a name that CMake uses to refer to some type of output
# In this case it is a library but it could also be an executable or some other items

# Public include directories can be used by other targets that link against turtlelib
# By adding include/ to the include path, this means that files in e.g., include/turtlelib
# can be included with #include"turtlelib/file.hpp"
# target_include_directories(turtlelib PUBLIC include/)

# enable C++ 17
target_compile_features(gaitlib PUBLIC cxx_std_17) 

# warnings are your friend!
target_compile_options(gaitlib PUBLIC -Wall -Wextra -Wpedantic)

if(NOT CMAKE_CROSSCOMPILING)

# stuff you don't want to be done when cross compiling
# CMake also has the ability to generate doxygen documentation
find_package(Doxygen)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
doxygen_add_docs(doxygen include/ src/ README.md ALL)

# Use the cmake testing functionality. A test is just an executable.
# We use the catch2 unit testing framework
find_package(Catch2 3 REQUIRED)

include(CTest)
# enable_testing()
add_executable(gaitlib_test tests/tests.cpp)
target_link_libraries(gaitlib_test gaitlib Catch2::Catch2WithMain)

add_test(NAME Test_of_Gaitlib COMMAND gaitlib_test)

endif()


# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# Using the library from the build directory or after installation
# During build, the headers are read from the source code directory
# When used from the installed location, headers are in the 
# system include/ directory
target_include_directories(gaitlib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# install the include directories
install(DIRECTORY include/gaitlib DESTINATION include)

# Install the targets and create a CMake Exported Target
# The CMake Exported Target can be used to access this project 
# from other CMake projects, after installation
# The targets will be installed to default locations
install(TARGETS gaitlib EXPORT gaitlib-targets)

# The project_name-targets now also needs to be exported.
# This call will generate a file called project_name-config.cmake
# That contains the exported targets.
# After installation this file will then be found when calling 
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use your library
install(EXPORT gaitlib-targets
        FILE gaitlib-config.cmake
        NAMESPACE gaitlib::
        DESTINATION lib/cmake/${PROJECT_NAME})