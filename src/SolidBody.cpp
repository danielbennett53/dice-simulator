#include "SolidBody.h"
#include <iostream>
#include <math.h>
#include <nlopt.hpp>

typedef struct {
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
} contact_problem_data;

typedef struct {
    int idx;
    int normal_idx;
    int sign;
} friction_constraint_data;

double myfunc(unsigned int n, const double *x, double *grad, void *my_func_data)
{
    auto *data = (contact_problem_data *) my_func_data;
    Eigen::VectorXd X(n);
    for (int i=0;i<n;++i) {
        X(i) = x[i];
    }

    if (grad) {
        Eigen::VectorXd g = 0.5*(data->A + data->A.transpose())*X + data->b;
        for (int i=0;i<n;++i) {
            grad[i] = g(i);
        }
    }


    double out = 0.5*X.dot(data->A*X) + data->b.dot(X);
    return out;
}

double friction_constraint(unsigned int n, const double *x, double *grad, void *in_data)
{
    auto *data = (friction_constraint_data *) in_data;
    double mu = 0.8;
    if (grad) {
        memset(grad, 0, sizeof(double)*n);
        grad[data->idx] = data->sign;
        grad[data->normal_idx] = -mu;
    }

    return (x[data->idx]*data->sign - mu*x[data->normal_idx]);
}

SolidBody::SolidBody(std::vector<Eigen::Vector3d> vertices,
                     std::vector<std::vector<unsigned int>> faces,
                     std::shared_ptr<Mesh> mesh)
{

    double m = 0.1; //kg
    M_.topLeftCorner(3,3) = (1.0/6.0) * m * 4.0 * Eigen::Matrix3d::Identity();
    M_.bottomRightCorner(3,3) = m * Eigen::Matrix3d::Identity();
    mesh_ = std::move(mesh);
    vertices_ = std::move(vertices);
    faces_ = std::move(faces);
}


void SolidBody::updatePosition()
{
    // Rotate orientation vector by current velocity
    Eigen::Quaterniond ang_vel;
    ang_vel.x() = vel_(0) * Ts_/2;
    ang_vel.y() = vel_(1) * Ts_/2;
    ang_vel.z() = vel_(2) * Ts_/2;
    ang_vel.w() = 1.0f;
    orientation_ *= ang_vel;
    orientation_.normalize();

    // Translate center of mass
    COM_(0) += vel_(3) * Ts_;
    COM_(1) += vel_(4) * Ts_;
    COM_(2) += vel_(5) * Ts_;

    mesh_->updateModelTF(COM_, orientation_);
}


Eigen::Matrix<double, Eigen::Dynamic, 6> SolidBody::getContactJacobian()
{
    Eigen::Matrix<double, Eigen::Dynamic, 6> Jc;
    long nrows = 0;

    for (const auto &vertex : vertices_) {
        Eigen::Quaterniond v_q;
        v_q.vec() = vertex;
        v_q.w() = 0;
        auto tfVertex_q = orientation_ * v_q * orientation_.inverse();
        auto tfVertex = tfVertex_q.vec();
        auto vertex_global = tfVertex + COM_;

        if (vertex_global(1) <= 0) {
            nrows += 3;
            Jc.conservativeResize(nrows, Eigen::NoChange_t());
            Eigen::Matrix3d r_cross;
            r_cross << 0, -tfVertex(2), tfVertex(1),
                       tfVertex(2), 0, -tfVertex(0),
                       -tfVertex(1), tfVertex(0), 0;
            Jc.bottomLeftCorner(3,3) = -r_cross;
            Jc.bottomRightCorner(3,3) = Eigen::Matrix3d::Identity();
        }
    }
    return Jc;
}


void SolidBody::step()
{
    Eigen::Matrix<double, Eigen::Dynamic, 6> Jc;
    Eigen::Matrix<double, Eigen::Dynamic, 1> x_err;
    double k = 70;
    double b = 2;
    long nrows = 0;

    for (const auto &vertex : vertices_) {
        Eigen::Quaterniond v_q;
        v_q.vec() = vertex;
        v_q.w() = 0;
        Eigen::Quaterniond tfVertex_q = orientation_ * v_q * orientation_.inverse();
        Eigen::Vector3d tfVertex = tfVertex_q.vec();
        Eigen::Vector3d vertex_global = tfVertex + COM_;

        if (vertex_global(1) <= 0) {
            nrows += 3;
            x_err.conservativeResize(nrows, Eigen::NoChange_t());
            x_err.bottomRows(3) << 0, vertex_global(1), 0;

            Jc.conservativeResize(nrows, Eigen::NoChange_t());
            Eigen::Matrix3d r_cross;
            r_cross << 0, -tfVertex(2), tfVertex(1),
                    tfVertex(2), 0, -tfVertex(0),
                    -tfVertex(1), tfVertex(0), 0;
            Jc.bottomLeftCorner(3, 3) = -r_cross;
            Jc.bottomRightCorner(3, 3) = Eigen::Matrix3d::Identity();
        }
    }

    // Dynamics with no impact
    Eigen::VectorXd g(6);
    g.setZero();
    g(4) = 9.81*M_(5,5);
    Eigen::VectorXd v_hat = vel_ - M_.inverse()*g*Ts_;

    auto size = Jc.rows();
    if (size > 0) {
        Eigen::VectorXd v_minus(size);
        v_minus = Jc * v_hat;
        Eigen::MatrixXd A(size,size);
        A = Jc * M_.inverse() * Jc.transpose();

        Eigen::VectorXd contact_vel(size);
        contact_vel = Jc * vel_;
        Eigen::VectorXd v_star(size);
        v_star = A * (-k * x_err - b * contact_vel) * Ts_ + contact_vel;
        Eigen::MatrixXd R(size,size);
        R = Eigen::MatrixXd::Identity(size, size) * 0.01;

        contact_problem_data prob_data = {
                .A = (A + R),
                .b = (v_minus - v_star)
        };

        double lb[size];
        for (int i=0;i<size;++i) {
            if ((i+2)%3 == 0) {
                lb[i] = 0;
            } else {
                lb[i] = -HUGE_VAL;
            }
        }

        nlopt_opt opt;
        opt = nlopt_create(NLOPT_LD_SLSQP, size);
        nlopt_set_lower_bounds(opt, lb);
        nlopt_set_min_objective(opt, myfunc, &prob_data);

        auto n_contacts = (size/3);
        friction_constraint_data constraint_data;

        for (int i=0;i<n_contacts;++i) {
            constraint_data = {
                    .idx = 3*i,
                    .normal_idx = 3*i+1,
                    .sign = 1
            };
            nlopt_add_inequality_constraint(opt, friction_constraint, &constraint_data, 1e-10);
            constraint_data = {
                    .idx = 3*i,
                    .normal_idx = 3*i+1,
                    .sign = -1
            };
            nlopt_add_inequality_constraint(opt, friction_constraint, &constraint_data, 1e-10);
            constraint_data = {
                    .idx = 3*i+2,
                    .normal_idx = 3*i+1,
                    .sign = 1
            };
            nlopt_add_inequality_constraint(opt, friction_constraint, &constraint_data, 1e-10);
            constraint_data = {
                    .idx = 3*i+2,
                    .normal_idx = 3*i+1,
                    .sign = -1
            };
            nlopt_add_inequality_constraint(opt, friction_constraint, &constraint_data, 1e-10);
        }

        nlopt_set_xtol_rel(opt, 1e-8);
        double x[size];
        for (auto state : x){
            state = 0;
        }

        double minf;
        if (nlopt_optimize(opt, x, &minf) < 0) {
            std::cout << "failed" << std::endl;
        }

        nlopt_destroy(opt);

        Eigen::VectorXd f(size);
        for (int i=0;i<f.rows();++i) {
            f(i) = x[i];
        }

//        std::cout << "f = " << f << std::endl;

        vel_ = v_hat + M_.inverse()*Jc.transpose()*f;
    } else {
        vel_ = v_hat;
    }
    updatePosition();
}