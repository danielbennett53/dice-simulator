#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include "../Mesh.h"
#include "../SolidBody.h"
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
    void addMesh(Mesh &m);
    void loadTextures(Mesh &m);
    void drawMesh(Mesh &m,
                  const Eigen::Transform<double, 3, Eigen::Affine> &mesh_tf =
                  Eigen::Transform<double, 3, Eigen::Affine>::Identity());
    void updateCameraView(void);
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void timerEvent(QTimerEvent *e) override;
    void initShaders();
    void printMatrix(const QMatrix4x4 &in);

    Qt::MouseButtons buttons_pressed_ = Qt::NoButton;
    std::vector<int> currMousePos_ = {0, 0, 2000};
    std::vector<int> lastMousePos_ = {0, 0, 2000};

    QOpenGLShaderProgram shader_;
    struct {
        GLint model_tf;
        GLint view_tf;
        GLint projection_tf;
        GLint texture;
    } uniforms_;

    QMatrix4x4 cam_view_;
    QMatrix4x4 projection_;

    std::vector<SolidBody> bodies_;

    QBasicTimer timer_;

};
