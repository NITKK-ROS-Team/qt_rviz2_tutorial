# cmakeの最小要件。ここでは、3.5以上を指定
cmake_minimum_required(VERSION 3.5)
# project名
project(rviz2panel_hello_template)

# UIファイルを自動で変換する設定
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake_auto REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

ament_auto_find_build_dependencies()

set(QT5_LIBS Qt5::Core Qt5::Gui Qt5::Widgets)

# ===== Widget =====
ament_auto_add_library(rviz2panel_hello_template_rviz SHARED
    src/widget/base/rviz2panel_hello_template.ui
    src/widget/base/base_widget.cpp
    src/widget/rviz_widget.cpp
)
target_link_libraries(rviz2panel_hello_template_rviz ${QT5_LIBS})
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

set(TARGET_WIDGET rviz2panel_hello_template_qt)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    # (あまり美しくないが)自動生成されたヘッダファイルのパスを追加
    ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_WIDGET}_autogen/include
)
ament_auto_add_library(${TARGET_WIDGET} SHARED
    src/widget/base/rviz2panel_hello_template.ui
    src/widget/base/base_widget.cpp
    src/widget/qt_widget.cpp
)
target_link_libraries(${TARGET_WIDGET} ${QT5_LIBS})
set(QT5_LIBS ${QT5_LIBS} ${TARGET_WIDGET})

# ==== MainWindow ====
ament_auto_add_library(rviz2panel_hello_template_mainwindow SHARED
    src/mainwindow/mainwindow.cpp)
target_link_libraries(rviz2panel_hello_template_mainwindow ${QT5_LIBS})
set(QT5_LIBS ${QT5_LIBS} rviz2panel_hello_template_mainwindow)

# ==== Executable ====
ament_auto_add_executable(rviz2panel_hello_template_exec
    src/main.cpp)
target_link_libraries(rviz2panel_hello_template_exec ${QT5_LIBS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()