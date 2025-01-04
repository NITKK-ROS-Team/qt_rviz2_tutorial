# 13. QtとROS Serviceの連携

Rviz2に埋めるService Clientのサンプルです。

Qtはタイマーやボタンの割り込み発生があり、ROSは非同期でメッセージを受信するため、両者の連携を行う際には注意が必要です。

このサンプルでは、Service Clientの実装を行い、Qtのタイマーを使って定期的にServiceを呼び出す方法を示します。

<br>

ROSのService Client実装にはQtのインタフェースは入っておらず、逆にQtのインタフェースにROSのService Client実装はないので、より簡潔に実装でき、ROS以外のインタフェースを実装する際に参考になると思います。

<br>

## ファイル構成

ROS 2に関連する実装は次のとおりです。

- `include/rviz2panel_service_client/qt_node_server_client_handler.hpp` : 
Service Clientのハンドラクラス
- `include/rviz2panel_service_client/ros_types.hpp` : ROSのメッセージ型の定義 (Service Clientのリクエスト/レスポンス)

なお、Widget側は `rviz_common::Panel` を継承することで、ROS 2の挙動を取り込んでいます。

```bash
.
├── CMakeLists.txt
├── include
│   └── rviz2panel_service_client
│       ├── qt_node_server_client_handler.hpp
│       └── qt_node_sub_handler.hpp
│       └── ros_types.hpp
├── package.xml
├── plugins_description.xml
├── README.md
└── src
    └── widget
        ├── rviz2panel_service_client.ui
        ├── widget.cpp
        └── widget.hpp
```

<br>

## Service Clientの実装

ウィジェットの実装において、Publisherの記述について示します。

### 初期化

```cpp
// onInitialize()内での初期化
qt_node_server_client_handler_.setRosNodePtr(this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
qt_node_server_client_handler_.initializeClient("add_two_ints");
```

### メッセージ送信

```cpp
void ExampleWidget::onPushButtonClicked()
{
  static uint32_t counter = 0;
  // サービスリクエストの作成, a = counter, b = counter + 1
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = counter;
  request->b = counter + 1;
  // サービスリクエストの送信
  qt_node_server_client_handler_.sendRequest(request);
  counter++;
}
```

<br>

### メッセージ受信

メッセージの受信を別で実装したため、直接メッセージ受信に反応させずに、タイマーで定期的にメッセージを受信するようにしています。

> [!NOTE]
> Widgetクラスと同じレイヤに書くことで、メッセージ受信時にUIの更新を行うこともできます。


```cpp
void ExampleWidget::onTimer()
{
  example_interfaces::srv::AddTwoInts::Response::SharedPtr msg;
  if (qt_node_server_client_handler_.getResponse(msg)) {
    // メッセージの表示
    uint32_t sum = msg->sum;
    std::string str = std::to_string((sum - 1) / 2) + " + " + std::to_string((sum - 1) / 2 + 1) + " = " + std::to_string(sum);
    ui.example_label->setText(QString::fromStdString(str));
  }
}
```

<br>


## Service Serverの起動

```bash
source /opt/ros/humble/setup.bash
ros2 run examples_rclcpp_minimal_service service_main
``` 
