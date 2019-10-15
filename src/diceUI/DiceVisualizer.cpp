#include "DiceVisualizer.h"

void DiceVisualizer::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.2f, 0x3f, 0x7f, 1.0f);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
}

void DiceVisualizer::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void DiceVisualizer::paintGL()
{
}

void DiceVisualizer::mouseMoveEvent(QMouseEvent* event)
{
    currMousePos_ = {event->x(), event->y()};
}

void DiceVisualizer::mousePressEvent(QMouseEvent* event)
{
    currMousePos_ = {event->x(), event->y()};
    mousePressed_ = true;
}

void DiceVisualizer::mouseReleaseEvent(QMouseEvent* event)
{
    currMousePos_ = {event->x(), event->y()};
    mousePressed_ = false;
}
