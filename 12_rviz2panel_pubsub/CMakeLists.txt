# cmakeの最小要件。ここでは、3.5以上を指定
cmake_minimum_required(VERSION 3.5)
# project名
project(rviz2panel_pubsub)

# UIファイルを自動で変換する設定
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake_auto REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

ament_auto_find_build_dependencies()

set(QT5_LIBS Qt5::Core Qt5::Gui Qt5::Widgets)

# ===== Widget =====
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
)
ament_auto_add_library(rviz2panel_pubsub SHARED
    src/widget/rviz2panel_pubsub.ui 
    src/widget/widget.cpp)
target_link_libraries(rviz2panel_pubsub ${QT5_LIBS})

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()