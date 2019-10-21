#include "DiceVisualizer.h"
#include <QtMath>
#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#include "../../include/stb_image.h"
#include <QMatrix4x4>
#include <QVector3D>
#include <QOpenGLVertexArrayObject>
#include <memory>
#include <QTimer>

void DiceVisualizer::initializeGL()
{
    makeCurrent();
    initializeOpenGLFunctions();
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    std::cout << QOpenGLContext::openGLModuleType() << std::endl;

    // Create shader
    shader_.addShaderFromSourceFile(QOpenGLShader::Vertex, QString("../shaders/vertex.glsl"));
    shader_.addShaderFromSourceFile(QOpenGLShader::Fragment, QString("../shaders/fragment.glsl"));
    shader_.bindAttributeLocation("pos_in", 0);
    shader_.bindAttributeLocation("tex_coord_in", 1);
    shader_.link();
    uniforms_.view_tf = shader_.attributeLocation("view");
    uniforms_.model_tf = shader_.attributeLocation("model");
    uniforms_.projection_tf = shader_.attributeLocation("proj");
    uniforms_.texture = shader_.attributeLocation("tex");
    shader_.bind();

    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_CULL_FACE);
    //glShadeModel(GL_SMOOTH);

    // Initialize Camera view
    cam_view_.setToIdentity();

    // Add all meshes
    for (auto &mesh : Mesh::options) {
        addMesh(mesh.second);
    }
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(100);
}

void DiceVisualizer::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void DiceVisualizer::paintGL()
{
    updateCameraView();
    makeCurrent();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    shader_.bind();
    QMatrix4x4 projection;

    projection.perspective(qDegreesToRadians(45.0f),
                           (static_cast<float>(width()) / height()), 0.1f, 100.0f);
    shader_.setUniformValue(uniforms_.view_tf, cam_view_);
    shader_.setUniformValue(uniforms_.projection_tf, projection);

    auto tf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    Eigen::Vector3d d = {-10, -10, -10};
    tf.translate(d);
    drawMesh(Mesh::options.at(Mesh::FLOOR), tf);
    for (const auto &body : bodies_) {
        drawMesh(Mesh::options.at(body.mesh_idx_), body.tf_);
    }
    shader_.release();
    this->context()->swapBuffers(this->context()->surface());
}

void DiceVisualizer::addMesh(Mesh &m)
{
    makeCurrent();
    m.VAO_ = std::make_unique<QOpenGLVertexArrayObject>();
    m.VBO_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
    m.EBO_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::IndexBuffer);
    m.VAO_->create();
    m.VBO_->create();
    m.EBO_->create();

    m.VAO_->bind();
    m.VBO_->bind();
    m.EBO_->bind();

    m.VBO_->setUsagePattern(QOpenGLBuffer::StaticDraw);
    m.VBO_->allocate(&m.vertices_[0], m.vertices_.size() * sizeof(Vertex));

    m.EBO_->setUsagePattern(QOpenGLBuffer::StaticDraw);
    m.EBO_->allocate(&m.indices_[0], m.indices_.size() * sizeof(int));

    // Define position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(0);
    // Define texture coordinates attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*) offsetof(Vertex, texCoords));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Load texture
    loadTextures(m);

    // Unbind vertex array
    m.VAO_->release();
    m.VBO_->release();
    m.EBO_->release();
}

void DiceVisualizer::loadTextures(Mesh &m)
{
    // Exit if no textures
    if (m.textureFilepath_.empty()) {
        std::cout << "No texture file found" << std::endl;
        return;
    }    
    auto tex_image = QImage(QString::fromStdString(m.textureFilepath_), ".png");
    m.tex_ = std::make_unique<QOpenGLTexture>(tex_image);
    m.tex_->create();
    m.tex_->setWrapMode(QOpenGLTexture::Repeat);
}

void DiceVisualizer::drawMesh(Mesh &m, const Eigen::Transform<double, 3, Eigen::Affine> &mesh_tf)
{
    makeCurrent();
    shader_.bind();
    // Activate texture and bind it
    m.tex_->bind();

    // Set model transform. Need to copy matrix for eigen -> QMatrix conversion
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> tf_copy = mesh_tf.matrix().cast<float>();
    auto t = QMatrix4x4();
    t.setToIdentity();
    shader_.setUniformValue(uniforms_.model_tf, t);//QMatrix4x4(tf_copy.data()));
    // draw mesh
    m.VAO_->bind();
    m.VBO_->bind();
    m.EBO_->bind();
    int loc = shader_.attributeLocation("pos_in");
    shader_.enableAttributeArray(loc);
    shader_.setAttributeBuffer(loc, GL_FLOAT, 0, 3, sizeof(Vertex));

    int loc2 = shader_.attributeLocation("tex_coord_in");
    shader_.enableAttributeArray(loc2);
    shader_.setAttributeBuffer(loc2, GL_FLOAT, sizeof(Vertex), 2, sizeof(Vertex));
    glDrawArrays(GL_TRIANGLES, 0, 10);
//    glDrawElements(GL_TRIANGLES, (GLsizei) m.indices_.size(), GL_UNSIGNED_INT,
//                   nullptr);
}

void DiceVisualizer::updateCameraView()
{
    // Store last position of mouse
    static double lastPosX, lastPosY = 0;
    // Difference of current and last position
    double diffX = currMousePos_[0] - lastPosX;
    double diffY = currMousePos_[1] - lastPosY;

    // Rotations about X and Y axes
    static double xAng;
    static double yAng = 10;

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
                        (float) sin(yAng * M_PI / 180.0f),
                        r * cos(xAng * M_PI / 180.0f));

    // Move camera target with right-click
    if (buttons_pressed_.testFlag(Qt::RightButton)) {
        right = QVector3D::crossProduct(cam_pos, up);
        right.normalize();
        cam_target += (right * (float) (diffX/50)) +
                  (QVector3D::crossProduct(right, cam_pos) * (float) (diffY/50));
    }

    // Calculate absolute camera position
    cam_pos = cam_pos * cam_target;
    // Don't let camera go below floor
    if (cam_pos[1] < 0.1) {
        cam_pos[1] = 0.1;
    }
    // Calculate camera view
    cam_view_.lookAt(cam_pos, cam_target, up);

    // Propagate mouse position history
    lastPosX = currMousePos_[0];
    lastPosY = currMousePos_[1];
}

void DiceVisualizer::mouseMoveEvent(QMouseEvent* event)
{
    currMousePos_ = {event->x(), event->y()};
}

void DiceVisualizer::mousePressEvent(QMouseEvent* event)
{
    currMousePos_ = {event->x(), event->y()};
    buttons_pressed_ |= event->button();
}

void DiceVisualizer::mouseReleaseEvent(QMouseEvent* event)
{
    currMousePos_ = {event->x(), event->y()};
    buttons_pressed_ ^= event->button();
}
