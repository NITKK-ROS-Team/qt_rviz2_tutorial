/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <QApplication>
#include "mainwindow/mainwindow.hpp"

int main(int argc, char * argv[])
{
  QApplication app(argc, argv);
  rviz2panel_hello_template::ExampleWindow window;
  window.show();
  return app.exec();
}
