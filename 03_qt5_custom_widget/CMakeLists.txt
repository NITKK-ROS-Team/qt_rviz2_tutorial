# cmakeの最小要件。ここでは、3.25以上を指定
cmake_minimum_required(VERSION 3.25)
# project名
project(qt5_custom_widget)

# UIファイルを自動で変換する設定
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)

# Qt5を探す (Core, Gui, Widgetsを使う)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

set(QT5_LIBS Qt5::Core Qt5::Gui Qt5::Widgets)

# ===== Custom Widget =====
add_library(custom_widget
    src/custom_widget/CanvasWidget/canvaswidget.cpp)
target_link_libraries(custom_widget ${QT5_LIBS})
set(QT5_LIBS ${QT5_LIBS} custom_widget)

# ===== Widget =====
set(TARGET_WIDGET qt5_custom_widget_widget)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    # 自作のWidgetのヘッダファイルのパスを追加
    ${CMAKE_CURRENT_SOURCE_DIR}/src/custom_widget/CanvasWidget/
    # (あまり美しくないが)自動生成されたヘッダファイルのパスを追加
    ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_WIDGET}_autogen/include
)
add_library(${TARGET_WIDGET}
    src/widget/qt5_custom_widget.ui 
    src/widget/widget.cpp
    ${WIDGETS}
)
target_link_libraries(${TARGET_WIDGET} ${QT5_LIBS})
set(QT5_LIBS ${QT5_LIBS} ${TARGET_WIDGET})

# ==== MainWindow ====
add_library(qt5_custom_widget_mainwindow
    src/mainwindow/mainwindow.cpp)
target_link_libraries(qt5_custom_widget_mainwindow ${QT5_LIBS})
set(QT5_LIBS ${QT5_LIBS} qt5_custom_widget_mainwindow)

# ==== Executable ====
add_executable(qt5_custom_widget_exec
    src/main.cpp)
target_link_libraries(qt5_custom_widget_exec ${QT5_LIBS})
