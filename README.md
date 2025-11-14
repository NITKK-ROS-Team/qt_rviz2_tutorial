# qt_rviz2_tutorial

[![ROS 2 CI (Multi-Distribution)](https://github.com/NITKK-ROS-Team/qt_rviz2_tutorial/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/NITKK-ROS-Team/qt_rviz2_tutorial/actions/workflows/ros2_ci.yml)
[![ROS 2 Jazzy CI](https://github.com/NITKK-ROS-Team/qt_rviz2_tutorial/actions/workflows/jazzy.yml/badge.svg)](https://github.com/NITKK-ROS-Team/qt_rviz2_tutorial/actions/workflows/jazzy.yml)

QtはC++を使用したGUI開発によく使われるツールキットであり、Rviz2のpanelに使用できます。

このチュートリアルでは、Qtの基本からRviz2プラグインの作成までを目標としています。

<br>

## サポートされているROS 2ディストリビューション

このプロジェクトは以下のROS 2ディストリビューションでテストされています：

- ROS 2 Humble Hawksbill (Ubuntu 22.04)
- ROS 2 Iron Irwini (Ubuntu 22.04)
- ROS 2 Jazzy Jalisco (Ubuntu 24.04)
- ROS 2 Rolling Ridley (Ubuntu 24.04)

CI（継続的インテグレーション）により、各ディストリビューションでのビルドとテストが自動的に実行されます。

<br>

## 対象

C++とCMakeListsが読める人。

<br>

## Qtを使用する意義

洗練されたGUIを作成可能なツールが多く増えた現在の開発環境ですが、それでもQtを採用する意義はあると考えています。

### 1. ROSではC++が主流であり、Rviz2ではQtが使用可能

ROS2においてはC++での開発が主流であり、GUIツールはRvizが多く使われています。

Rviz2はROS2のGUIツールとして多くの機能を提供していますが、その中でPanelとしてQtを使用することで、Rviz2の機能を拡張することができます。

そのため、ROS2開発に関わる人の基本的なスキルセットにはほとんどの場合にC++が含まれるため、最初からC++を使用可能なQtを使用することは自然な選択肢と言えます。

### 2. Qtはプラットフォーム・世代を超えてAPIが安定している

1991年に登場したQtは、その後多くのバージョンアップ・ライセンス変更を経て、現在はQt6がリリースされています。

Windows・MacOSX・Linuxのほかにも組み込み向け（商用版）など、多くのプラットフォームに対応しており、それらに対する基本的なAPIの一貫性や安定性が高く、長期的な開発・保守に適しています。

オープンソースライセンス版も、商用品質に近い安定性を持っており、多くのOSSプロジェクトで使用されています。

<br>

## コンテンツ

### 1 ~ : Qt5の基本

1. [Qt5 Hello World!](./01_qt5_hello_world/README.md)
2. [Layout・ファイル分割](./02_qt5_layout/README.md)
3. [ウィジェットのカスタマイズ](./03_qt5_custom_widget/RREADME.md)

### 10 ~ : Rviz2プラグインの作成

10. [Rviz2への移植](./10_rviz2panel_hello/README.md)
11. [Rviz2 <--> Qtウィンドウの内部実装の共通化](./11_rviz2panel_hello_template/README.md)
12. [QtとROS Pub-Sub](./12_rviz2panel_pubsub/README.md)
13. [QtとROS Serviceの連携](./13_rviz2panel_service_client/README.md)
14. [QtとROS Actionの連携](./14_rviz2panel_action_client/README.md)


### 20 ~ : その他
20. [QtウィジェットのROS-LikeなCMakelists.txtの作成](./20_qt5_hello_ros2_cmake/README.md)
