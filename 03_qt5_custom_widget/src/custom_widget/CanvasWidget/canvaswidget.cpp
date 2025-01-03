/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "custom_widget/CanvasWidget/canvaswidget.h"

CanvasWidget::CanvasWidget(QWidget * parent)
: QLabel(parent)
{
  loaded_image_ = QPixmap(100, 100);
  loaded_image_.fill(Qt::white);
  setPixmap(loaded_image_);
}

void CanvasWidget::drawPoint(int x, int y)
{
  QImage image = loaded_image_.toImage();
  image.setPixelColor(x, y, Qt::black);

  setPixmap(QPixmap::fromImage(image));
  loaded_image_ = QPixmap::fromImage(image);
}

void CanvasWidget::reset(const uint32_t size)
{
  loaded_image_ = QPixmap(size, size);
  loaded_image_.fill(Qt::white);
  setPixmap(loaded_image_);
}

void CanvasWidget::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    drawPoint(event->x(), event->y());
  }
}

void CanvasWidget::mouseMoveEvent(QMouseEvent * event)
{
  if (event->buttons() & Qt::LeftButton) {
    drawPoint(event->x(), event->y());
  }
}
