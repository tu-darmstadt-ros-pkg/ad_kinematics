cmake_minimum_required(VERSION 3.16)
project(ad_kinematics LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    Eigen3
    rclcpp
    geometry_msgs  	
    urdf
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########

add_library(ad_kinematics SHARED
  src/transforms.cpp
  src/tree.cpp
  src/urdf_loader.cpp
)

target_compile_features(ad_kinematics PUBLIC cxx_std_17)

target_include_directories(ad_kinematics PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/ad_kinematics>
)

ament_target_dependencies(ad_kinematics PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ad_kinematics PUBLIC Eigen3::Eigen)

install(
  FILES     
    include/ad_kinematics/transforms.h
    include/ad_kinematics/tree.h
    include/ad_kinematics/urdf_loader.h
  DESTINATION include/ad_kinematics
)
install(
  TARGETS
    ad_kinematics
  EXPORT export_ad_kinematics
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_tree.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_export_targets(export_ad_kinematics HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()