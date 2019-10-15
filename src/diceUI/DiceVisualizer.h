#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <vector>

class DiceVisualizer : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    DiceVisualizer(QWidget *parent) : QOpenGLWidget(parent) {}

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

    bool mousePressed_ = false;
    std::vector<int> lastMousePos_ = {0, 0};
    std::vector<int> currMousePos_ = {0, 0};
};
