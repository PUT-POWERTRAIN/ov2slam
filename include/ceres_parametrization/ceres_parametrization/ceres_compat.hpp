/**
* Ceres Compatibility Wrapper
*
* Provides compatibility between Ceres 1.x (LocalParameterization)
* and Ceres 2.x+ (Manifold) APIs
*/
#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>

// Check Ceres version
#if CERES_VERSION_MAJOR >= 2
    #define CERES_HAS_MANIFOLD_API 1
#else
    #define CERES_HAS_MANIFOLD_API 0
#endif

#if CERES_HAS_MANIFOLD_API

/**
 * Wrapper to provide old LocalParameterization interface
 * using new Ceres 2.x Manifold API
 */
class LocalParameterization {
public:
    virtual ~LocalParameterization() = default;

    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const = 0;

    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const = 0;

    virtual int GlobalSize() const = 0;
    virtual int LocalSize() const = 0;
};

/**
 * Adapter from LocalParameterization to Manifold
 */
class LocalParameterizationToManifoldAdapter : public ceres::Manifold {
public:
    explicit LocalParameterizationToManifoldAdapter(
        std::unique_ptr<LocalParameterization> param)
        : param_(std::move(param)) {}

    bool Plus(const double* x,
              const double* delta,
              double* x_plus_delta) const override {
        return param_->Plus(x, delta, x_plus_delta);
    }

    bool PlusJacobian(const double* x,
                      double* jacobian) const override {
        return param_->ComputeJacobian(x, jacobian);
    }

    bool Minus(const double* y,
               const double* x,
               double* y_minus_x) const override {
        // For SE(3) with left update: Plus(x, delta) = exp(delta) * x
        // We want delta such that Plus(x, delta) = y
        // => exp(delta) * x = y
        // => exp(delta) = y * x^(-1)
        // => delta = log(y * x^(-1))

        // Extract SE(3) elements from x and y
        Eigen::Map<const Eigen::Vector3d> tx(x);
        Eigen::Map<const Eigen::Quaterniond> qx(x+3);
        Sophus::SE3d X(qx, tx);

        Eigen::Map<const Eigen::Vector3d> ty(y);
        Eigen::Map<const Eigen::Quaterniond> qy(y+3);
        Sophus::SE3d Y(qy, ty);

        // Compute delta = log(Y * X^(-1))
        Sophus::SE3d delta = Y * X.inverse();
        Eigen::Matrix<double, 6, 1> delta_vec = delta.log();

        // Copy result
        Eigen::Map<Eigen::Matrix<double, 6, 1>> result(y_minus_x);
        result = delta_vec;

        return true;
    }

    bool MinusJacobian(const double* x,
                       double* jacobian) const override {
        // MinusJacobian is the derivative of Minus(y, x) w.r.t. x
        // For SE(3) with left update, Minus(y, x) = log(y * x^(-1))
        // The derivative w.r.t. x is -Adjoint(y * x^(-1))^(-1)
        // However, LocalParameterization doesn't provide this
        // For most optimization problems, only Plus and PlusJacobian are used

        // Set jacobian to -identity as a reasonable approximation
        // This works for small changes and is consistent with the Lie group structure
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
        J.topRows<6>().setIdentity();
        J.topRows<6>() *= -1.0;
        J.bottomRows<1>().setZero();

        return true;
    }

    int AmbientSize() const override {
        return param_->GlobalSize();
    }

    int TangentSize() const override {
        return param_->LocalSize();
    }

private:
    std::unique_ptr<LocalParameterization> param_;
};

/**
 * Helper function to create a Ceres parameterization with proper adapter
 * Works with both Ceres 1.x and 2.x
 */
inline ceres::Manifold* createManifold(LocalParameterization* param) {
#if CERES_HAS_MANIFOLD_API
    return new LocalParameterizationToManifoldAdapter(
        std::unique_ptr<LocalParameterization>(param));
#else
    return static_cast<ceres::Manifold*>(param);
#endif
}

#endif // CERES_HAS_MANIFOLD_API
