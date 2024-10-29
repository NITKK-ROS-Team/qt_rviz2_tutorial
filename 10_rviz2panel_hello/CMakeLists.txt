# cmakeの最小要件。ここでは、3.25以上を指定
cmake_minimum_required(VERSION 3.25)
# project名
project(rviz2panel_hello)

# UIファイルを自動で変換する設定
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

set(QT5_LIBS Qt5::Core Qt5::Gui Qt5::Widgets)

# ===== Widget =====
set(TARGET_WIDGET rviz2panel_hello_widget)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    # (あまり美しくないが)自動生成されたヘッダファイルのパスを追加
    ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_WIDGET}_autogen/include
)
add_library(${TARGET_WIDGET}
    src/widget/rviz2panel_hello.ui 
    src/widget/widget.cpp)
target_link_libraries(${TARGET_WIDGET} ${QT5_LIBS})

# ==== MainWindow ====
add_library(rviz2panel_hello_mainwindow
    src/mainwindow/mainwindow.cpp)
target_link_libraries(rviz2panel_hello_mainwindow rviz2panel_hello_widget ${QT5_LIBS})
ament_target_dependencies(rviz2panel_hello_mainwindow rviz_common rviz_rendering)

pluginlib_export_plugin_description_file(rviz_common rviz2panel_hello_mainwindow.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()