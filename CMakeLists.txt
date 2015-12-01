cmake_minimum_required(VERSION 2.8.3)
project(ur_modern_driver C CXX)


#
# Test if this build using ROS/catkin
#
function (is_catkin_build RESULT)
  set (${RESULT} FALSE PARENT_SCOPE)
  if (DEFINED ENV{ROS_ROOT})
    message (WARNING "Assuming library is built using ROS/catkin since ROS_ROOT is defined in environment")
    if (DEFINED CATKIN_DEVEL_PREFIX)
      set (${RESULT} TRUE PARENT_SCOPE)
    else ()
      message (WARNING "CATKIN_DEVEL_PREFIX is not defined so assuming this is NOT a ROS/catkin build after all")
    endif ()
  endif ()
endfunction (is_catkin_build)


# Set CATKIN_IS_BUILT
is_catkin_build (CATKIN_IS_BUILT)


# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()
#




## Declare a C++ executable
set(${PROJECT_NAME}_SOURCES
    src/ur_realtime_communication.cpp
    src/ur_communication.cpp
    src/robot_state.cpp
    src/robot_state_RT.cpp
    src/do_output.cpp
    src/ur_driver.cpp
    src/ur_hardware_interface_standalone.cpp
)
    
set(${PROJECT_NAME}_HEADERS
    include/ur_modern_driver/do_output.h
    include/ur_modern_driver/portable_endian.h
    include/ur_modern_driver/robot_state.h
    include/ur_modern_driver/robot_state_RT.h
    include/ur_modern_driver/ur_communication.h
    include/ur_modern_driver/ur_driver.h
    include/ur_modern_driver/ur_hardware_interface.h
    include/ur_modern_driver/ur_hardware_interface_standalone.h
    include/ur_modern_driver/ur_realtime_communication.h
)

# Destination for CMake package config files
if (CATKIN_IS_BUILT)
  include_directories(include)
  # Output directories
  set (PACKAGE_CONFIG_DIRECTORY "${CATKIN_DEVEL_PREFIX}/cmake")
  set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")

  add_definitions( -DROS_BUILD )


    # Find catkin macros and libraries
    # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
    # is used, also find other catkin packages
    find_package(catkin REQUIRED COMPONENTS
      hardware_interface
      controller_manager
      actionlib
      control_msgs
      geometry_msgs
      roscpp
      sensor_msgs
      std_msgs
      trajectory_msgs
      ur_msgs
      tf
    )
    
    ## System dependencies are found with CMake's conventions
    # find_package(Boost REQUIRED COMPONENTS system)
    
    
    ## Uncomment this if the package has a setup.py. This macro ensures
    ## modules and global scripts declared therein get installed
    ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
    # catkin_python_setup()
    
    ################################################
    ## Declare ROS messages, services and actions ##
    ################################################
    
    ## To declare and build messages, services or actions from within this
    ## package, follow these steps:
    ## * Let MSG_DEP_SET be the set of packages whose message types you use in
    ##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
    ## * In the file package.xml:
    ##   * add a build_depend tag for "message_generation"
    ##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
    ##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
    ##     but can be declared for certainty nonetheless:
    ##     * add a run_depend tag for "message_runtime"
    ## * In this file (CMakeLists.txt):
    ##   * add "message_generation" and every package in MSG_DEP_SET to
    ##     find_package(catkin REQUIRED COMPONENTS ...)
    ##   * add "message_runtime" and every package in MSG_DEP_SET to
    ##     catkin_package(CATKIN_DEPENDS ...)
    ##   * uncomment the add_*_files sections below as needed
    ##     and list every .msg/.srv/.action file to be processed
    ##   * uncomment the generate_messages entry below
    ##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)
    
    ## Generate messages in the 'msg' folder
    # add_message_files(
    #   FILES
    #   Message1.msg
    #   Message2.msg
    # )
    
    ## Generate services in the 'srv' folder
    # add_service_files(
    #   FILES
    #   Service1.srv
    #   Service2.srv
    # )
    
    ## Generate actions in the 'action' folder
    # add_action_files(
    #   FILES
    #   Action1.action
    #   Action2.action
    # )
    
    ## Generate added messages and services with any dependencies listed here
    # generate_messages(
    #   DEPENDENCIES
    #   control_msgs#   geometry_msgs#   sensor_msgs#   trajectory_msgs#   ur_msgs
    # )
    
    ################################################
    ## Declare ROS dynamic reconfigure parameters ##
    ################################################
    
    ## To declare and build dynamic reconfigure parameters within this
    ## package, follow these steps:
    ## * In the file package.xml:
    ##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
    ## * In this file (CMakeLists.txt):
    ##   * add "dynamic_reconfigure" to
    ##     find_package(catkin REQUIRED COMPONENTS ...)
    ##   * uncomment the "generate_dynamic_reconfigure_options" section below
    ##     and list every .cfg file to be processed
    
    ## Generate dynamic reconfigure parameters in the 'cfg' folder
    # generate_dynamic_reconfigure_options(
    #   cfg/DynReconf1.cfg
    #   cfg/DynReconf2.cfg
    # )
    
    ###################################
    ## catkin specific configuration ##
    ###################################
    ## The catkin_package macro generates cmake config files for your package
    ## Declare things to be passed to dependent projects
    ## INCLUDE_DIRS: uncomment this if you package contains header files
    ## LIBRARIES: libraries you create in this project that dependent projects also need
    ## CATKIN_DEPENDS: catkin_packages dependent projects also need
    ## DEPENDS: system dependencies of this project that dependent projects also need
    catkin_package(
      INCLUDE_DIRS include
    #  LIBRARIES ur_modern_driver
      CATKIN_DEPENDS hardware_interface controller_manager actionlib control_msgs geometry_msgs roscpp sensor_msgs trajectory_msgs ur_msgs
      DEPENDS ur_hardware_interface
    )

    #
    # ###########
    # ## Build ##
    # ###########
    #
    ## Add cmake target dependencies of the library
    ## as an example, code may need to be generated before libraries
    ## either from message generation or dynamic reconfigure
    add_dependencies(ur_modern_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    
    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    include_directories(include
      ${catkin_INCLUDE_DIRS}
    )

    ## Declare a C++ library

    # Hardware Interface
    add_library(ur_hardware_interface src/ur_hardware_interface_ros.cpp)
    target_link_libraries(ur_hardware_interface_ros
      ${catkin_LIBRARIES}
    )

    add_library(ur_driver ${${PROJECT_NAME}_SOURCES})
    
    add_executable(ur_driver_ros 
        src/ur_ros_wrapper.cpp
    )
    
    target_link_libraries(ur_driver_ros ur_driver)
    ## Add cmake target dependencies of the executable
    ## same as for the library above
    add_dependencies(ur_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ur_driver
      ur_hardware_interface
      ${catkin_LIBRARIES}
     )

    #############
    ## Install ##
    #############
    
     install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
    
     ## Mark executables and/or libraries for installation
     install(TARGETS ur_driver ur_hardware_interface
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     )
    
     ## Mark cpp header files for installation
     install(DIRECTORY include/${PROJECT_NAME}/
       DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       FILES_MATCHING PATTERN "*.h"
     )
    
    # Mark other files for installation (e.g. launch and bag files, etc.)
     install(FILES
       # myfile1
       # myfile2
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
     )
    
    #############
    ## Testing ##
    #############
    
    ## Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test test/test_ur_modern_driver.cpp)
    if(TARGET ${PROJECT_NAME}-test)
      target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
    endif()
    
    ## Add folders to be run by python nosetests
    catkin_add_nosetests(test)

else (CATKIN_IS_BUILT)

    ########################################################################
    ######################## Standard CMake Build ##########################
    ########################################################################

    ########################### Misc. Configs ##############################
    # Add custom cmake files folder
    SET (PROJECT_CMAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
    # This sets where to look for modules (e.g. "Find*.cmake" files)
    SET (CMAKE_MODULE_PATH "${PROJECT_CMAKE_DIR}" "${CMAKE_MODULE_PATH}" )

    # Add custom Install Settings
    INCLUDE(InstallSettings)
    

    find_package(Boost)

    if(ros_control_FOUND)
      # todo: consider adding https://github.com/ros-controls/ros_control
      # it can be modified quite easily to not require ros
      #include_directories(ros_control/hardware_interface/include)  
      add_executable(ur_driver_standalone src/ur_hardware_interface_standalone.cpp)
    endif(ros_control_FOUND)
    


    ################ Installation project target ################

    set(TARGET_NAME ${PROJECT_NAME})
    add_custom_target(${TARGET_NAME} ALL)
    include(GenerateExportHeader)
    
    ######################### Add Primary Targets ##########################
    #ADD_SUBDIRECTORY(src)

    add_library(ur_driver
        ${${PROJECT_NAME}_SOURCES}
        ${${PROJECT_NAME}_HEADERS}
      )

    add_dependencies(${TARGET_NAME} ur_driver)

    target_include_directories(ur_driver PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>  # <prefix>/include
    )
    target_include_directories(ur_driver PRIVATE ${Boost_INCLUDE_DIRS} )

    ######################### Installation Stuff ###########################
    include(GenerateExportHeader)
    generate_export_header(ur_driver)

    install(TARGETS ur_driver
      EXPORT ${TARGET_NAME}Targets
      RUNTIME DESTINATION ${INSTALL_BIN_DIR}
      LIBRARY DESTINATION ${INSTALL_LIB_DIR}
      ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
      INCLUDES DESTINATION ${INSTALL_INCLUDE_DIR}
    )

    install(
      FILES
        ${${PROJECT_NAME}_HEADERS}
      DESTINATION
        ${INSTALL_INCLUDE_DIR}
      COMPONENT
        Devel
    )

    ########################## Add the Examples ############################
    # OPTION (BUILD_EXAMPLES "Build Examples" ON)
    # IF(BUILD_EXAMPLES)
    #   ADD_SUBDIRECTORY(examples)
    # ENDIF()

    ######################### Installation Stuff ###########################
    INCLUDE(InstallProjectConfig)

    ###################### Add uninstall target ############################
    ADD_CUSTOM_TARGET(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${PROJECT_CMAKE_DIR}/cmake_uninstall.cmake)

    ##################### Show Configuration Summary ######################
    MESSAGE(STATUS "===============================================================")
    MESSAGE(STATUS "================  Configuration Summary  ======================")
    MESSAGE(STATUS "Project Name:       ${PROJECT_NAME}")
    MESSAGE(STATUS "Build type:         ${CMAKE_BUILD_TYPE}")
    MESSAGE(STATUS "Build type Flags:   ${CMAKE_BUILD_TYPE_FLAGS}")
    MESSAGE(STATUS "C++ compile flags:  ${CMAKE_CXX_FLAGS}")
    MESSAGE(STATUS "Install Path:       ${CMAKE_INSTALL_PREFIX}")
    MESSAGE(STATUS "Build Examples:     ${BUILD_EXAMPLES}")
endif(CATKIN_IS_BUILT)

