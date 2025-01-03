/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RVIZ2PANEL_HELLO_WIDGET_QT_WIDGET_HPP_
#define RVIZ2PANEL_HELLO_WIDGET_QT_WIDGET_HPP_

#include "widget/base/base_widget.hpp"

namespace rviz2panel_hello_template
{

// Widgetのクラス
class ExampleWidget : public BaseExampleWidget<QWidget>
{
public:
  // 初期化
  explicit ExampleWidget(QWidget * parent = nullptr);
};

} // namespace rviz2panel_hello_template

#endif //RVIZ2PANEL_HELLO_WIDGET_QT_WIDGET_HPP_
