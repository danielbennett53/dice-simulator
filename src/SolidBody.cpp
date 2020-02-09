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

double contact_cost_fcn(unsigned int n, const double *x, double *grad, void *my_func_data)
{
    auto *data = (contact_problem_data *) my_func_data;
    Eigen::VectorXd X(n);
    for (unsigned int i=0;i<n;++i) {
        X(i) = x[i];
    }

    if (grad) {
        Eigen::VectorXd g = 0.5*(data->A + data->A.transpose())*X + data->b;
        for (unsigned int i=0;i<n;++i) {
            grad[i] = g(i);
        }
    }

    double out = 0.5*X.dot(data->A*X) + data->b.dot(X);
    return out;
}

double friction_constraint(unsigned int n, const double *x, double *grad, void *in_data)
{
    auto *data = (friction_constraint_data *) in_data;
    double mu = 0.7;
    if (grad) {
        memset(grad, 0, sizeof(double)*n);
        grad[data->idx] = data->sign;
        grad[data->normal_idx] = -mu;
    }

    return (x[data->idx]*data->sign - mu*x[data->normal_idx]);
}


SolidBody::SolidBody(std::shared_ptr<geometry::ConvexPolytope> shape, double density)
{
    shape_ = std::move(shape);
    vel_ = Eigen::Matrix<double, 6, 1>::Zero();
    calculatePhysicalProperties(density);
}

void SolidBody::setPosition(Eigen::Transform<double, 3, Eigen::Affine> tf)
{
    shape_->setTransform(tf);
//    orientation_ = tf.rotation();
}


void SolidBody::updatePosition()
{
    // Rotate orientation vector by current velocity
    Eigen::Quaterniond ang_vel;
    ang_vel.vec() = vel_.head(3) * Ts_/2;
    ang_vel.w() = 1.0f;
//    orientation_ = ang_vel*orientation_;
//    orientation_.normalize();

//    // Translate center of mass
//    COM_ += vel_.tail(3) * Ts_;
//    tf_ = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
//    tf_.translate(COM_ + COM_offset_);
//    tf_.rotate(orientation_);
    auto tf_temp = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    Eigen::Vector3d t = vel_.tail(3) * Ts_/2;
    tf_temp.translate(t);
    tf_temp.rotate(ang_vel);
    shape_->transform(tf_temp);
}

void SolidBody::step()
{
    Eigen::Matrix<double, Eigen::Dynamic, 6> Jc;
    Eigen::Matrix<double, Eigen::Dynamic, 1> x_err;
    double e = 0.2;
    double tau = 0.0017;
    double B = 2*(1+e)*(1/tau);
    double K = (1+e)/(tau*tau);
    long nrows = 0;
    for (const auto &vertex : std::dynamic_pointer_cast<geometry::ConvexPolytope>(shape_)->getVertices()) {
        auto vtx = vertex->getPos();
        auto vtx_rel = vtx - shape_->getCentroid();

        if (vtx(1) <= 0) {
            nrows += 3;
            x_err.conservativeResize(nrows, Eigen::NoChange_t());
            x_err.bottomRows(3) << 0, vtx(1), 0;

            Jc.conservativeResize(nrows, Eigen::NoChange_t());
            Eigen::Matrix3d r_cross;
            r_cross << 0, -vtx_rel(2), vtx_rel(1),
                    vtx_rel(2), 0, -vtx_rel(0),
                    -vtx_rel(1), vtx_rel(0), 0;
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
        contact_vel = Jc * v_hat;
        Eigen::VectorXd v_star(size);
        v_star = (-K * x_err - B * v_minus) * Ts_ + contact_vel;
        Eigen::MatrixXd R(size,size);
        R = Eigen::MatrixXd::Identity(size, size);
        for ( int j=0; j<size; ++j) {
            double a_jj = A(j,j)*e;
            R(j,j) = a_jj < 0.07 ? 0.07 : a_jj;
        }

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
        nlopt_set_min_objective(opt, contact_cost_fcn, &prob_data);

        auto n_contacts = (size/3);

        for (int i=0;i<n_contacts;++i) {
            friction_constraint_data constraint_data[4];
            constraint_data[0] = {
                    .idx = 3*i,
                    .normal_idx = 3*i+1,
                    .sign = 1
            };
            nlopt_add_inequality_constraint(opt, friction_constraint, &constraint_data[0], 1e-10);
            constraint_data[1] = {
                    .idx = 3*i,
                    .normal_idx = 3*i+1,
                    .sign = -1
            };
            nlopt_add_inequality_constraint(opt, friction_constraint, &constraint_data[1], 1e-10);
            constraint_data[2] = {
                    .idx = 3*i+2,
                    .normal_idx = 3*i+1,
                    .sign = 1
            };
            nlopt_add_inequality_constraint(opt, friction_constraint, &constraint_data[2], 1e-10);
            constraint_data[3] = {
                    .idx = 3*i+2,
                    .normal_idx = 3*i+1,
                    .sign = -1
            };
            nlopt_add_inequality_constraint(opt, friction_constraint, &constraint_data[3], 1e-10);
        }

        nlopt_set_xtol_rel(opt, 1e-8);
        double x[size];
        for (auto& state : x){
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

        vel_ = v_hat + M_.inverse()*Jc.transpose()*f;
    } else {
        vel_ = v_hat;
    }
    updatePosition();
}


void SolidBody::calculatePhysicalProperties(float density)
{
    // Calculate centroid and volume
    float volume = 0.0;
    auto centroid = Eigen::Vector3d(0.0, 0.0, 0.0);
    M_ = Eigen::Matrix<double, 6, 6>::Zero();

    auto temp_shape = std::dynamic_pointer_cast<geometry::ConvexPolytope>(shape_);
    // Pick first vertex as starting point to make sure it is within the mesh
    auto refPoint = temp_shape->getVertices()[0]->getPos();

    // Calculate volume and centroid
    for (const auto& face : temp_shape->faces_) {
        Eigen::Vector3d a = face.edges_[0].startPoint_->getPos();
        Eigen::Vector3d b = face.edges_[1].startPoint_->getPos();
        Eigen::Vector3d c = face.edges_[2].startPoint_->getPos();

        double temp_volume = fabs(((a - refPoint).cross(b - refPoint)).dot(c - refPoint) / 6.0);
        auto temp_centroid = Eigen::Vector3d(a[0] + b[0] + c[0] + refPoint[0],
                                       a[1] + b[1] + c[1] + refPoint[1],
                                       a[2] + b[2] + c[2] + refPoint[2]) * 0.25f;

        // update centroid if volume of new segment is nonzero
        if (temp_volume > 0) {
            centroid = volume * centroid + temp_volume * temp_centroid;
            volume += temp_volume;
            centroid = centroid / volume;
        }
    }

    COM_offset_ << centroid[0], centroid[1], centroid[2];

    float Ia = 0.0;
    float Ib = 0.0;
    float Ic = 0.0;
    float Ia_p = 0.0;
    float Ib_p = 0.0;
    float Ic_p = 0.0;

    // Calculate intertia tensor
    for (const auto& face : temp_shape->faces_) {
        Eigen::Vector3d a = face.edges_[0].startPoint_->getPos() - centroid;
        Eigen::Vector3d b = face.edges_[1].startPoint_->getPos() - centroid;
        Eigen::Vector3d c = face.edges_[2].startPoint_->getPos() - centroid;

        double vol = fabs(a.cross(b).dot(c)) / 6.0f;
        Ia += density * vol *
                (a[1]*a[1] + a[1]*b[1] + b[1]*b[1] + a[1]*c[1] + b[1]*c[1] +
                 c[1]*c[1] + a[2]*a[2] + a[2]*b[2] + b[2]*b[2] + a[2]*c[2] +
                 b[2]*c[2] + c[2]*c[2]) / 10.0f;
        Ib += density * vol *
                (a[0]*a[0] + a[0]*b[0] + b[0]*b[0] + a[0]*c[0] + b[0]*c[0] +
                 c[0]*c[0] + a[2]*a[2] + a[2]*b[2] + b[2]*b[2] + a[2]*c[2] +
                 b[2]*c[2] + c[2]*c[2]) / 10.0f;
        Ic += density * vol *
                (a[0]*a[0] + a[0]*b[0] + b[0]*b[0] + a[0]*c[0] + b[0]*c[0] +
                 c[0]*c[0] + a[1]*a[1] + a[1]*b[1] + b[1]*b[1] + a[1]*c[1] +
                 b[1]*c[1] + c[1]*c[1]) / 10.0f;
        Ia_p += density * vol *
                (2*a[1]*a[2] + b[1]*a[2] + c[1]*a[2] + a[1]*b[2] +
                 2*b[1]*b[2] + c[1]*b[2] + a[1]*c[2] + b[1]*c[2] +
                 2*c[1]*c[2]) / 20.0f;
        Ib_p += density * vol *
                (2*a[0]*a[2] + b[0]*a[2] + c[0]*a[2] + a[0]*b[2] +
                 2*b[0]*b[2] + c[0]*b[2] + a[0]*c[2] + b[0]*c[2] +
                 2*c[0]*c[2]) / 20.0f;
        Ic_p += density * vol *
                (2*a[0]*a[1] + b[0]*a[1] + c[0]*a[1] + a[0]*b[1] +
                 2*b[0]*b[1] + c[0]*b[1] + a[0]*c[1] + b[0]*c[1] +
                 2*c[0]*c[1]) / 20.0f;
    }

    Eigen::Matrix3d I;
    I << Ia, -Ib_p, -Ic_p,
        -Ib_p, Ib, -Ia_p,
        -Ic_p, -Ia_p, Ic;
    M_.topLeftCorner(3,3) = I;
    M_.bottomRightCorner(3,3) = density * volume * Eigen::Matrix3d::Identity();
}
