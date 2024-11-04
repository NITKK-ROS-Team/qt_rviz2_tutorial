# cmakeの最小要件。ここでは、3.5以上を指定
cmake_minimum_required(VERSION 3.5)
# project名
project(qt5_layout)

# UIファイルを自動で変換する設定
set(CMAKE_AUTOUIC ON)

# Qt5を探す (Core, Gui, Widgetsを使う)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

set(QT5_LIBS Qt5::Core Qt5::Gui Qt5::Widgets)

# ===== Widget =====
set(TARGET_WIDGET qt5_layout_widget)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    # (あまり美しくないが)自動生成されたヘッダファイルのパスを追加
    ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_WIDGET}_autogen/include
)
add_library(${TARGET_WIDGET}
    src/widget/qt5_layout.ui 
    src/widget/widget.cpp)
target_link_libraries(${TARGET_WIDGET} ${QT5_LIBS})
set(QT5_LIBS ${QT5_LIBS} ${TARGET_WIDGET})

# ==== MainWindow ====
add_library(qt5_layout_mainwindow
    src/mainwindow/mainwindow.cpp)
target_link_libraries(qt5_layout_mainwindow ${QT5_LIBS})
set(QT5_LIBS ${QT5_LIBS} qt5_layout_mainwindow)

# ==== Executable ====
add_executable(qt5_layout_exec
    src/main.cpp)
target_link_libraries(qt5_layout_exec ${QT5_LIBS})
