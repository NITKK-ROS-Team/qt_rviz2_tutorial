# 12. QtとROS Pub-Sub

Rviz2に埋めるPub-Subのサンプルです。

Qtはタイマーやボタンの割り込み発生があり、ROSはSubscriberがメッセージを受信するとコールバック関数が呼ばれるため、両者を組み合わせるときには注意が必要です。

このサンプルでは、Qtのタイマー駆動を主軸として、ROSのSubscriber処理を実装します。

<br>

ROSのPub-Sub実装にはQtのインタフェースは入っておらず、逆にQtのインタフェースにROSのPub-Sub実装はないので、より簡潔に実装でき、ROS以外のインタフェースを実装する際に参考になると思います。

<br>

## ファイル構成

ROS 2に関連する実装は次のとおりです。

- `include/rviz2panel_pubsub/qt_node_pub_handler.hpp` : Publisherのハンドラクラス
- `include/rviz2panel_pubsub/qt_node_sub_handler.hpp` : Subscriberのハンドラクラス
- `include/rviz2panel_pubsub/ros_types.hpp` : ROSのメッセージ型の定義 (PublisherとSubscriberの共通ヘッダファイル)

なお、Widget側は `rviz_common::Panel` を継承することで、ROS 2の挙動を取り込んでいます。

```bash
.
├── CMakeLists.txt
├── include
│   └── rviz2panel_pubsub
│       ├── qt_node_pub_handler.hpp
│       └── qt_node_sub_handler.hpp
│       └── ros_types.hpp
├── package.xml
├── plugins_description.xml
├── README.md
└── src
    └── widget
        ├── rviz2panel_pubsub.ui
        ├── widget.cpp
        └── widget.hpp
```

<br>

## Publisherの実装

ウィジェットの実装において、Publisherの記述について示します。

### 初期化

```cpp
// onInitialize()内での初期化
qt_node_pub_handler_.setRosNodePtr(this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
qt_node_pub_handler_.initializePublisher("example_topic");
```

### メッセージ送信

```cpp
void ExampleWidget::onPushButtonClicked()
{
  static uint32_t counter = 0;
  // パブリッシャを使ってメッセージを送信
  std_msgs::msg::String msg;
  msg.data = std::to_string(++counter);
  qt_node_pub_handler_.publishMsg(msg);
}
```

<br>

## Subscriberの実装

ウィジェットの実装において、Subscriberの記述について示します。

### 初期化

```cpp
// onInitialize()内での初期化
qt_node_sub_handler_.setRosNodePtr(this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
qt_node_sub_handler_.initializeSubscription("example_topic");
```

### メッセージ受信

メッセージの受信を別で実装したため、直接メッセージ受信に反応させずに、タイマーで定期的にメッセージを受信するようにしています。

> [!NOTE]
> Widgetクラスと同じレイヤに書くことで、メッセージ受信時にUIの更新を行うこともできます。


```cpp
void ExampleWidget::onTimer()
{
  // サブスクライバを使ってメッセージを受信
  std_msgs::msg::String msg;
  if (qt_node_sub_handler_.getMsg(msg))
  {
    ui_.label->setText(QString::fromStdString(msg.data));
  }
}
```

<br>