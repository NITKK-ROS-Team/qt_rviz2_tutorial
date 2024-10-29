/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <QApplication>
#include <QMainWindow>

// 作成した.uiファイル名によって変わる
// xxx.ui -> ui_xxx.h
#include "ui_qt5_hello_world.h"


// Widgetのクラス
class ExampleWidget : public QWidget
{
public:
  // 初期化
  explicit ExampleWidget(QWidget * parent = nullptr)
  : QWidget(parent)
  {
    // UIの初期化
    ui.setupUi(this);
    // ボタンがクリックされたときの処理を設定
    connect(
      ui.example_push_button, &QPushButton::clicked, this,
      &ExampleWidget::onPushButtonClicked);
  }

private:
  // ボタンがクリックされたときの処理
  void onPushButtonClicked()
  {
    static uint32_t counter = 0;
    // カウンタをインクリメントしてラベルに表示
    ui.example_label->setText(QString("%1").arg(++counter));
  }

private:
  Ui::ExampleWidget ui;
};


// MainWindowのクラス
class ExampleWindow : public QMainWindow
{
public:
  explicit ExampleWindow(QWidget * parent = nullptr)
  : QMainWindow(parent)
  {
    // タイトルの設定
    setWindowTitle("Hello, Qt5!");
    // サイズの設定
    resize(800, 600);

    // ウィジェットの設定（中央に配置して新規作成）
    setCentralWidget(new ExampleWidget(this));
  }
};


// メイン関数
int main(int argc, char * argv[])
{
  // アプリケーションの初期化(ほとんどテンプレ)
  QApplication app(argc, argv);
  ExampleWindow window;
  window.show();
  return app.exec();
}
