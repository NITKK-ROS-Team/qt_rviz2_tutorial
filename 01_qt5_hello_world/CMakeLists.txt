# cmakeの最小要件。ここでは、3.5以上を指定
cmake_minimum_required(VERSION 3.5)
# project名
project(qt5_hello_world)

# UIファイルを自動で変換する設定
set(CMAKE_AUTOUIC ON)

# Qt5を探す (Core, Gui, Widgetsを使う)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# `qt5_hello_world_exec`という実行ファイルを作成
# - src/qt5_hello_world.uiとsrc/main.cppを使う
# - (uiファイルは自動で変換される)
add_executable(qt5_hello_world_exec
    src/qt5_hello_world.ui 
    src/main.cpp)

# `qt5_hello_world_exec` に対してリンクするライブラリを指定
# - ここではQt5::Core, Qt5::Gui, Qt5::Widgetsを使用
target_link_libraries(qt5_hello_world_exec
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
)
