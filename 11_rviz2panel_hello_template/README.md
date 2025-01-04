# 11. Rviz2 <--> Qtウィンドウの内部実装の共通化
Rviz2PanelはQtウィジェットを継承しているため、Qtウィンドウと同じように内部実装を共通化することができます。

ここでは、Rviz2PanelとQtウィンドウの内部実装を共通化する方法を紹介します。

>  そもそもQtウィンドウとRviz2Panelの内部実装の共通化が必要なのか？という疑問があるかもしれませんが...

<br>

## 1. Rviz2Panelの内部実装の共通部分

今回作成する共通部分は、 `widget/base` ディレクトリに配置します。

```bash
├── CMakeLists.txt
├── package.xml
├── plugins_description.xml
├── README.md
└── src
    ├── main.cpp
    ├── mainwindow
    │   ├── mainwindow.cpp
    │   └── mainwindow.hpp
    └── widget
        ├── base
        │   ├── base_widget.cpp
        │   ├── base_widget.hpp
        │   └── rviz2panel_hello_template.ui
        ├── qt_widget.cpp
        ├── qt_widget.hpp
        └── rviz_widget.cpp

4 directories, 13 files
```

`base_widget.cpp` と `base_widget.hpp` は、Rviz2PanelとQtウィンドウの内部実装の共通部分を記述します。

```cpp
// base_widget.cpp

#include "widget/base/base_widget.hpp"

namespace rviz2panel_hello_template
{

template <class T>
BaseExampleWidget<T>::BaseExampleWidget(QWidget * parent)
: T(parent)
{
  // UIの初期化
  ui.setupUi(this);
  // ボタンがクリックされたときの処理を設定
  this->connect(ui.example_push_button, &QPushButton::clicked, this, &BaseExampleWidget::onPushButtonClicked);
}

template <class T>
void BaseExampleWidget<T>::onPushButtonClicked()
{
  static uint32_t counter = 0;
  // カウンタをインクリメントしてラベルに表示
  ui.example_label->setText(QString("%1").arg(++counter));
}

// エラー回避のため、明示的なインスタンス化
template class BaseExampleWidget<QWidget>;
template class BaseExampleWidget<rviz_common::Panel>;

} // namespace rviz2panel_hello_template

```

```cpp
// base_widget.hpp

#ifndef RVIZ2PANEL_HELLO_TEMPLATE_BASE_WIDGET_HPP_
#define RVIZ2PANEL_HELLO_TEMPLATE_BASE_WIDGET_HPP_

// 作成した.uiファイル名によって変わる
// xxx.ui -> ui_xxx.h
#include "ui_rviz2panel_hello_template.h"

// Q_MOC_RUN is defined when this file is processed by moc
#ifndef Q_MOC_RUN
#include <rviz_common/panel.hpp>

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#endif

#include <QObject>
#include <QWidget>


namespace rviz2panel_hello_template
{

// Widgetのクラス
template <class T>
class BaseExampleWidget : public T
{
public:
  // 初期化
  explicit BaseExampleWidget(QWidget *);

private:
  // ボタンがクリックされたときの処理
  void onPushButtonClicked();

private:
  Ui::ExampleWidget ui;
};

} // namespace rviz2panel_hello_template

#endif //RVIZ2PANEL_HELLO_TEMPLATE_BASE_WIDGET_HPP_
```

### ポイント: テンプレートクラスの定義

`BaseExampleWidget` クラスはテンプレートクラスとして定義されています。テンプレートクラスは、クラスの定義時に型を指定することができるクラスです。この場合、 `QWidget` または `rviz_common::Panel` を継承するクラスを指定することができます。

`rviz_common::Panel` 内部のオーバーライド可能なメソッドについては、もう一つ上のレイヤで実装できるので、base_widget.hpp には記述しません。
