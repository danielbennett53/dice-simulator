#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include "../SolidBody.h"
#include "ConvexPolytope.h"
#include "Shape.h"
#include "ObjReader.h"
#include <vector>
#include <memory>
#include <QBasicTimer>

class DiceVisualizer : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    DiceVisualizer(QWidget *parent);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void updateCameraView(void);
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void timerEvent(QTimerEvent *e) override;
    void initShaders();
    void printMatrix(const QMatrix4x4 &mat, const std::string &name);
    QMatrix4x4 eigenTFToQMatrix4x4(Eigen::Transform<double, 3, Eigen::Affine> &in);
    QVector4D eigenToQVector4d(Eigen::Vector3d &in);


    Qt::MouseButtons buttons_pressed_ = Qt::NoButton;
    std::vector<int> currMousePos_ = {0, 0, 2000};
    std::vector<int> lastMousePos_ = {0, 0, 2000};

    QOpenGLShaderProgram shader_;
    struct {
        GLint model_tf;
        GLint view_tf;
        GLint projection_tf;
        GLint texture;
        GLint color;
    } uniforms_;

    QMatrix4x4 cam_view_;
    QMatrix4x4 projection_;
    int w_, h_;
    bool paused_ = true;

    std::vector<SolidBody> bodies_;
    std::unique_ptr<geometry::Shape> shape;
    QBasicTimer timer_;
};
