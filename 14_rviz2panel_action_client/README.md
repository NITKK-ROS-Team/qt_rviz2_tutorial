# 14. QtとROS Actionの連携

Rviz2に埋めるAction Clientのサンプルです。

Qtはタイマーやボタンの割り込み発生があり、ROSは非同期でメッセージを受信するため、両者の連携を行う際には注意が必要です。

このサンプルでは、Action Clientの実装を行い、Qtのタイマーを使って定期的にActionを呼び出す方法を示します。

<br>

ROSのAction Client実装にはQtのインタフェースは入っておらず、逆にQtのインタフェースにROSのAction Client実装はないので、より簡潔に実装でき、ROS以外のインタフェースを実装する際に参考になると思います。

<br>

## ファイル構成

ROS 2に関連する実装は次のとおりです。

- `include/rviz2panel_action_client/qt_node_action_client_handler.hpp` : 
Action Clientのハンドラクラス
- `include/rviz2panel_action_client/ros_types.hpp` : ROSのメッセージ型の定義 (Action Clientのリクエスト/レスポンス)

なお、Widget側は `rviz_common::Panel` を継承することで、ROS 2の挙動を取り込んでいます。

```bash
.
├── CMakeLists.txt
├── include
│   └── rviz2panel_action_client
│       ├── qt_node_action_client_handler.hpp
│       └── qt_node_sub_handler.hpp
│       └── ros_types.hpp
├── package.xml
├── plugins_description.xml
├── README.md
└── src
    └── widget
        ├── rviz2panel_action_client.ui
        ├── widget.cpp
        └── widget.hpp
```

<br>

## Action Clientの実装

ウィジェットの実装において、Publisherの記述について示します。

### 初期化

```cpp
// onInitialize()内での初期化
qt_node_action_client_handler_.setRosNodePtr(
  this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
qt_node_action_client_handler_.initializeActionClient("fibonacci");
```

### メッセージ送信

```cpp
void ExampleWidget::onPushButtonClicked()
{
  static uint32_t order = 0;
  // アクションゴールの作成
  example_interfaces::action::Fibonacci::Goal goal;
  goal.order = order;
  // アクションゴールの送信
  qt_node_action_client_handler_.sendGoal(goal);
  order++;
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
  std::shared_ptr<const example_interfaces::action::Fibonacci::Result> msg_goal;
  std::shared_ptr<const example_interfaces::action::Fibonacci::Feedback> msg_feedback;

  if (qt_node_action_client_handler_.getResult(msg_goal)) {
    // メッセージの表示
    std::string str = "Result: (" + std::to_string(msg_goal->sequence.size()) + ") = [";
    for (size_t i = 0; i < msg_goal->sequence.size(); i++) {
      str += std::to_string(msg_goal->sequence[i]);
      if (i != msg_goal->sequence.size() - 1) {
        str += ", ";
      }
    }
    str += "]";
    ui.result_label->setText(QString::fromStdString(str));
    std::cout << str << std::endl;

  } else if (qt_node_action_client_handler_.getLatestFeedback(msg_feedback)) {
    std::string str = "Feedback: (" + std::to_string(msg_feedback->sequence.size()) + ") = [";
    for (size_t i = 0; i < msg_feedback->sequence.size(); i++) {
      str += std::to_string(msg_feedback->sequence[i]);
      if (i != msg_feedback->sequence.size() - 1) {
        str += ", ";
      }
    }
    str += "]";
    ui.feedback_label->setText(QString::fromStdString(str));
    std::cout << str << std::endl;
  }
}
```

<br>