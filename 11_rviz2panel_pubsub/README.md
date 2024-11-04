# QtとROS Pub-Sub

Rviz2に埋めるPub-Subのサンプルです。

Qtはタイマーやボタンの割り込み発生があり、ROSはSubscriberがメッセージを受信するとコールバック関数が呼ばれるため、両者を組み合わせるときには注意が必要です。

このサンプルでは、Qtのタイマー駆動を主軸として、ROSのSubscriber処理を実装します。



## ファイル構成

ROS 2に関連する実装は次のとおりです。

- `qt_node_pub_handler.hpp` : Publisherのハンドラクラス
- `qt_node_sub_handler.hpp` : Subscriberのハンドラクラス


```bash
.
├── CMakeLists.txt
├── include
│   └── rviz2panel_pubsub
│       ├── qt_node_pub_handler.hpp
│       └── qt_node_sub_handler.hpp
├── package.xml
├── plugins_description.xml
├── README.md
└── src
    └── widget
        ├── rviz2panel_pubsub.ui
        ├── widget.cpp
        └── widget.hpp

4 directories, 9 files
```