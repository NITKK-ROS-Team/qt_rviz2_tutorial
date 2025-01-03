/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "widget/base/base_widget.hpp"

namespace rviz2panel_hello_template
{

// Widgetのクラス
class ExampleWidget : public BaseExampleWidget<rviz_common::Panel>
{
public:
  // 初期化
  explicit ExampleWidget(QWidget * parent = nullptr)
  : BaseExampleWidget(parent) {}

  // rviz pluginのオーバーライド
  void onInitialize() override
  {
    std::cout << "Hello, rviz2panel_hello_template!" << std::endl;
  }
};

} // namespace rviz2panel_hello_template

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2panel_hello_template::ExampleWidget, rviz_common::Panel)
