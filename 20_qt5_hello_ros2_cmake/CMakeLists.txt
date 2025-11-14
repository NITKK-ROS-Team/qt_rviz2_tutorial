# cmakeの最小要件。ここでは、3.5以上を指定
cmake_minimum_required(VERSION 3.5)
# project名
project(qt5_hello_ros2_cmake)

# UIファイルを自動で変換する設定
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake_auto REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# amentのビルド依存関係を取得 (ROS2 pkgの依存関係を自動で解決できるので便利)
ament_auto_find_build_dependencies()

set(QT5_LIBS Qt5::Core Qt5::Gui Qt5::Widgets)

# ===== Widget =====
set(TARGET_WIDGET qt5_hello_ros2_cmake_widget)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    # (あまり美しくないが)自動生成されたヘッダファイルのパスを追加
    ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_WIDGET}_autogen/include
)
add_library(${TARGET_WIDGET}
    src/widget/qt5_hello_ros2_cmake.ui
    src/widget/widget.cpp)
target_link_libraries(${TARGET_WIDGET} ${QT5_LIBS})
set(QT5_LIBS ${QT5_LIBS} ${TARGET_WIDGET})

# ==== MainWindow ====
add_library(qt5_hello_ros2_cmake_mainwindow
    src/mainwindow/mainwindow.cpp)
target_link_libraries(qt5_hello_ros2_cmake_mainwindow ${QT5_LIBS})
set(QT5_LIBS ${QT5_LIBS} qt5_hello_ros2_cmake_mainwindow)

# ==== Executable ====
# add_executable(qt5_hello_ros2_cmake_exec
#     src/main.cpp)
# add_executableと同じ使い方でament_auto_add_executableを使う
ament_auto_add_executable(qt5_hello_ros2_cmake_exec
    src/main.cpp)
target_link_libraries(qt5_hello_ros2_cmake_exec ${QT5_LIBS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# amentのパッケージ情報を生成
ament_auto_package()