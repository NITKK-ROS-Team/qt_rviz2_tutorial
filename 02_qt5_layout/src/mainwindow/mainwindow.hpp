/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef QT5_LAYOUT_MAINWINDOW_MAINWINDOW_HPP
#define QT5_LAYOUT_MAINWINDOW_MAINWINDOW_HPP

#include <QMainWindow>
#include "widget/widget.hpp"

class ExampleWindow : public QMainWindow{
public:
    explicit ExampleWindow(QWidget * = nullptr);
};

#endif //QT5_LAYOUT_MAINWINDOW_MAINWINDOW_HPP