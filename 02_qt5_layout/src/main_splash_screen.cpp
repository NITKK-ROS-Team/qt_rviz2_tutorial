/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <QApplication>
#include "mainwindow/mainwindow.hpp"
#include <QSplashScreen>
#include <QThread>
#include <QTimer>
#include <filesystem>

int main(int argc, char * argv[])
{
  QApplication app(argc, argv);

  std::string path = std::filesystem::current_path();
  QPixmap loading((path + "/loading_window.png").c_str());
  loading = loading.scaled(480, 320, Qt::KeepAspectRatio);
  QSplashScreen splash(loading);
  splash.show();


  qt5_layout::ExampleWindow window;

  QTimer::singleShot(1000, &splash, SLOT(close()));
  QTimer::singleShot(1000, &window, SLOT(show()));

  return app.exec();
}
