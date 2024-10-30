/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CUSTOM_WIDGET_IMAGELABEL_IMAGE_LABEL_HPP_
#define CUSTOM_WIDGET_IMAGELABEL_IMAGE_LABEL_HPP_

#include <QLabel>
#include <QPixmap>
#include <QImage>
#include <QMouseEvent>


class CanvasWidget : public QLabel{
public:
    explicit CanvasWidget(QWidget *parent);
    ~CanvasWidget() = default;

    void drawPoint(int x, int y);
    void reset(const uint32_t size);

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

private:
    QPixmap loaded_image_;
    QLabel *image_window_;
};


#endif // CUSTOM_WIDGET_IMAGELABEL_IMAGE_LABEL_HPP_