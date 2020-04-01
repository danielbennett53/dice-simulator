#include "DiceVisualizer.h"
#include "Geometry.h"
#include "Plane.h"
#include "ObjReader.h"
#include <QtMath>
#include <iostream>
#include <QMatrix4x4>
#include <QVector3D>
#include <QOpenGLVertexArrayObject>
#include <memory>
#include <QTimer>

DiceVisualizer::DiceVisualizer(QWidget *parent) : QOpenGLWidget(parent) {}

void DiceVisualizer::timerEvent(QTimerEvent *e)
{
    (void) e;
    update();
}

void DiceVisualizer::printMatrix(const QMatrix4x4 &mat, const std::string &name)
{
    std::cout << name << std::endl;
    std::cout << mat(0,0) << " | " << mat(0,1) << " | " <<
                 mat(0,2) << " | " << mat(0,3) << std::endl;
    std::cout << mat(1,0) << " | " << mat(1,1) << " | " <<
                 mat(1,2) << " | " << mat(1,3) << std::endl;
    std::cout << mat(2,0) << " | " << mat(2,1) << " | " <<
                 mat(2,2) << " | " << mat(2,3) << std::endl;
    std::cout << mat(3,0) << " | " << mat(3,1) << " | " <<
                 mat(3,2) << " | " << mat(3,3) << std::endl << std::endl;

}

void DiceVisualizer::initializeGL()
{
    initializeOpenGLFunctions();
    glFrontFace(GL_CW);
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;

    setFocusPolicy(Qt::ClickFocus);
    glClearColor(1, 1, 1, 1);

    initShaders();

    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);
    glDepthMask(true);

    makeCurrent();
    shape = std::make_unique<geometry::Plane>(
                ObjReader("/home/daniel/github/dice-simulator/resources/floor.obj"));
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    QSurfaceFormat format;
    format.setVersion(4,5);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setSamples(0);
    QSurfaceFormat::setDefaultFormat(format);

    auto t1 = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    auto t2 = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    auto t3 = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    t1.translate(Eigen::Vector3d(-3.0, 4.0, 0.0));
    Eigen::AngleAxisd rot1(0.75, Eigen::Vector3d(1, 0.5, 0));
    t1 = t1 * rot1;
    t2.translate(Eigen::Vector3d(0.0, 4.0, 0.0));
    t2 = t2*rot1;
    t3.translate(Eigen::Vector3d(3.0, 4.0, 0.0));
    bodies_.emplace_back(std::make_shared<geometry::ConvexPolytope>(ObjReader("../../resources/d4.obj")));
    bodies_.emplace_back(std::make_shared<geometry::ConvexPolytope>(ObjReader("../../resources/d6.obj")));
    bodies_.emplace_back(std::make_shared<geometry::ConvexPolytope>(ObjReader("../../resources/d20.obj")));
   bodies_[0].shape_->transform(t1);
//    std::dynamic_pointer_cast<geometry::ConvexPolytope>(bodies_[1].shape_)->transform(t2);
//    std::dynamic_pointer_cast<geometry::ConvexPolytope>(bodies_[2].shape_)->transform(t3);
//    bodies_[1].vel_ << 1.0, 2.0, 4.0, 0.5, 0.3, 0.1;
    // Use QBasicTimer because its faster than QTimer
    timer_.start(12, this);
}

void DiceVisualizer::initShaders()
{
    // Compile vertex shader
    if (!shader_.addShaderFromSourceFile(QOpenGLShader::Vertex, "../shaders/vertex.glsl"))
        close();

    // Compile fragment shader
    if (!shader_.addShaderFromSourceFile(QOpenGLShader::Fragment, "../shaders/fragment.glsl"))
        close();

    // Link shader pipeline
    if (!shader_.link())
        close();

    // Bind shader pipeline for use
    if (!shader_.bind())
        close();

    uniforms_.model_tf = shader_.uniformLocation("model");
    uniforms_.projection_tf = shader_.uniformLocation("projection");
    uniforms_.view_tf = shader_.uniformLocation("view");
    uniforms_.texture = shader_.uniformLocation("tex");
    uniforms_.color = shader_.uniformLocation("color");

    shader_.release();
}

void DiceVisualizer::resizeGL(int w, int h)
{
    // Calculate aspect ratio
    qreal aspect = qreal(w) / qreal(h ? h : 1);

    // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
    const qreal zNear = 0.1, zFar = 100.0, fov = 45.0;

    w_ = w;
    h_ = h;

    // Reset projection
    projection_.setToIdentity();

    // Set perspective projection
    projection_.perspective(fov, aspect, zNear, zFar);
}

void DiceVisualizer::paintGL()
{
    if (!paused_) {
        for (int i = 0; i < 12; ++i) {
            for (auto &body : bodies_) {
                body.step();
            }
        }
    }

    makeCurrent();
    updateCameraView();
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_CULL_FACE);
    glClearColor(0.2f, 0.3f, 0.7f, 1.0f);
    shader_.bind();

    shader_.setUniformValue(uniforms_.view_tf, cam_view_);
    shader_.setUniformValue(uniforms_.projection_tf, projection_);
    shader_.setUniformValue(uniforms_.texture, 0);
    QMatrix4x4 tf;
    tf.setToIdentity();
    shader_.setUniformValue(uniforms_.model_tf, tf);
    shader_.setUniformValue(uniforms_.color, QVector4D{1.0, 1.0, 1.0, 1.0});
    shape->draw(shader_);

    for (auto& b : bodies_) {
        if (b.selected_)
            shader_.setUniformValue(uniforms_.color, QVector4D{0.0, 0.7, 0.3, 1.0});
        else
            shader_.setUniformValue(uniforms_.color, QVector4D{1.0, 1.0, 1.0, 1.0});
        tf.setToIdentity();
        shader_.setUniformValue(uniforms_.model_tf,tf);
        b.shape_->draw(shader_);
    }
    shader_.release();
}

void DiceVisualizer::updateCameraView()
{
    // Difference of current and last position
    double diffX = currMousePos_[0] - lastMousePos_[0];
    double diffY = currMousePos_[1] - lastMousePos_[1];

    // Rotations about X and Y axes
    static double xAng;
    static double yAng = 20;

    // Vectors for defining camera view
    static auto cam_target = QVector3D(0.0f, 0.0f, 0.0f);
    QVector3D cam_pos, up, right;

    // Rotation using middle mouse button
    if(buttons_pressed_.testFlag(Qt::MidButton)) {
        xAng += diffX / 5;
        yAng += diffY / 5;
    }

    // Keep angles within +- 180 degrees
    if (yAng > 180)
        yAng -= 360;
    if (yAng < -180)
        yAng += 360;
    if (xAng > 180)
        xAng -= 360;
    if (xAng < -180)
        xAng += 360;

    // Flip up vector when y angle goes beyond +/- 90
    if (yAng < 90 && yAng > -90) {
        up = QVector3D(0.0f, 1.0f, 0.0f);
    } else {
        up = QVector3D(0.0f, -1.0f, 0.0f);
    }

    // Spherical coordinates calculation for rotation:
    // Radius of projected circle
    auto r = (float) cos(yAng * M_PI / 180.0f);
    // Calculate x, y, z coords of camera position
    cam_pos = QVector3D(r * sin(xAng * M_PI / 180.0f),
                        sin(yAng * M_PI / 180.0f),
                        r * cos(xAng * M_PI / 180.0f));

    // Move camera target with right-click
    if (buttons_pressed_.testFlag(Qt::RightButton)) {
        right = QVector3D::crossProduct(cam_pos, up);
        right.normalize();
        cam_target += (right * (float) (diffX/50)) +
                  (QVector3D::crossProduct(right, cam_pos) * (float) (diffY/50));
    }

    // Calculate absolute camera position
    cam_pos = cam_pos * (currMousePos_[2]/100)  + cam_target;
    // Don't let camera go below floor
    if (cam_pos[1] < 0.1) {
        cam_pos[1] = 0.1;
    }
    // Calculate camera view
    cam_view_.setToIdentity();
    cam_view_.lookAt(cam_pos, cam_target, up);

    // Move dice
    if (buttons_pressed_.testFlag(Qt::LeftButton)) {
        for (auto& b : bodies_) {
            if (b.selected_) {
                auto translate = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
                Eigen::Matrix3d rot;
                auto R = cam_view_.inverted();
                rot << R(0,0), R(0,1), R(0,2),
                       R(1,0), R(1,1), R(2,2),
                       R(2,0), R(2,1), R(2,2);

                translate.translate(rot * Eigen::Vector3d(0.01 * diffX, -0.01 * diffY, 0));
                b.shape_->transform(translate);
            }
        }
    }

    // Propagate mouse position history
    lastMousePos_ = currMousePos_;
}

void DiceVisualizer::mouseMoveEvent(QMouseEvent* event)
{
    currMousePos_[0] = event->x();
    currMousePos_[1] = event->y();
}

void DiceVisualizer::mousePressEvent(QMouseEvent* event)
{
    makeCurrent();
    currMousePos_[0] = event->x();
    currMousePos_[1] = event->y();
    lastMousePos_ = currMousePos_;
    buttons_pressed_ |= event->button();
    if (event->buttons().testFlag(Qt::LeftButton)){
        auto x = (double) 2*(currMousePos_[0] - w_/2) / w_;
        auto y = (double) 2*(-currMousePos_[1] + h_/2) / h_;
        // Define ray from screen to max view
        auto origin = QVector4D(x, y, -1.0, 1.0)*0.1;
        auto p2 = QVector4D(x, y, 1.0, 1.0)*100;

        origin = cam_view_.inverted() * projection_.inverted() * (origin);
        p2 = cam_view_.inverted() * projection_.inverted() * (p2);
        QVector4D direction = p2 - origin;
        direction.normalize();
        Eigen::Vector3d intersection;
        int idx = -1;
        double dist = HUGE_VAL;
        for (unsigned int i = 0; i < bodies_.size(); ++i) {
            auto start = Eigen::Vector4d(origin[0], origin[1], origin[2], 1.0);
            auto dir = Eigen::Vector4d(direction[0], direction[1], direction[2], 0.0);

            if (bodies_[i].shape_->rayIntersection(
                        start.head(3),
                        dir.head(3),
                        intersection)) {
                intersection = (Eigen::Vector4d(intersection[0],
                                intersection[1], intersection[2], 1.0)).head(3);
                if (intersection.dot(dir.head(3)) < dist) {
                    dist = intersection.dot(dir.head(3));
                    idx = i;
                }
            }
        }
        if (idx >= 0) {
            bodies_[idx].selected_ = !bodies_[idx].selected_;
        }
    }
}

void DiceVisualizer::mouseReleaseEvent(QMouseEvent* event)
{
    currMousePos_[0] = event->x();
    currMousePos_[1] = event->y();
    buttons_pressed_ ^= event->button();
}

void DiceVisualizer::wheelEvent(QWheelEvent *event)
{
    currMousePos_[0] = event->x();
    currMousePos_[1] = event->y();
    currMousePos_[2] -= event->angleDelta().y();
}

void DiceVisualizer::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Delete) {
        for (int i = bodies_.size() - 1; i >= 0; --i) {
            if (bodies_[i].selected_)
                bodies_.erase(bodies_.begin() + i);
        }
    }
    if (event->key() == Qt::Key_Space) {
        paused_ = !paused_;
    }
}

QVector4D DiceVisualizer::eigenToQVector4d(Eigen::Vector3d &in)
{
    return QVector4D(in[0], in[1], in[2], in[3]);
}
