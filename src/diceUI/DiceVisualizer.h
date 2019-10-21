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

class DiceVisualizer : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    DiceVisualizer(QWidget *parent) : QOpenGLWidget(parent) {}

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

    Qt::MouseButtons buttons_pressed_ = Qt::NoButton;
    std::vector<int> lastMousePos_ = {0, 0};
    std::vector<int> currMousePos_ = {0, 0};

    QOpenGLShaderProgram shader_;
    struct {
        GLint model_tf;
        GLint view_tf;
        GLint projection_tf;
        GLint texture;
    } uniforms_;

    QMatrix4x4 cam_view_;

    std::vector<SolidBody> bodies_;

    QOpenGLVertexArrayObject VAO_;
    QOpenGLBuffer VBO_, EBO_;
};
