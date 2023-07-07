#pragma once

#include "Eigen/Eigen"

class FirstOrderFilter {
private:
  double ts_;

  Eigen::VectorXf initFn2_;
  Eigen::VectorXf initFn3_;
  Eigen::VectorXf initFn4_;
  Eigen::VectorXf delta1_;
  Eigen::VectorXf delta2_;
  Eigen::VectorXf delta3_;
  Eigen::VectorXf delta4_;
  Eigen::VectorXf yt_;

  float pole_;
  float gain_;
  Eigen::VectorXf initFn_;

public:
  FirstOrderFilter() {}
  ~FirstOrderFilter() {}

  void initializeFilter(float t, float gn, float pl, Eigen::VectorXf initFnVal) {
    ts_ = t;
    gain_ = gn;
    pole_ = pl;

    initFn_.resize(initFnVal.rows(), initFnVal.cols());
    initFn2_.resize(initFnVal.rows(), initFnVal.cols());
    initFn3_.resize(initFnVal.rows(), initFnVal.cols());
    initFn4_.resize(initFnVal.rows(), initFnVal.cols());

    delta1_.resize(initFnVal.rows(), initFnVal.cols());
    delta2_.resize(initFnVal.rows(), initFnVal.cols());
    delta3_.resize(initFnVal.rows(), initFnVal.cols());
    delta4_.resize(initFnVal.rows(), initFnVal.cols());

    yt_.resize(initFnVal.rows(), initFnVal.cols());
    initFn_ = initFnVal;
  }

  Eigen::VectorXf functionDot(float gn, float pl, const Eigen::VectorXf& initFnVal, const Eigen::VectorXf& fnT) {
    return -pl * initFnVal + gn * fnT;
  }

  // Compute the integral of first order differential eq. using RK4
  Eigen::VectorXf getRK4Integral(const Eigen::VectorXf& fnT) {
    delta1_ = ts_ * functionDot(gain_, pole_, initFn_, fnT);
    initFn2_ = initFn_ + 0.5 * delta1_;
    delta2_ = ts_ * functionDot(gain_, pole_, initFn2_, fnT);
    initFn3_ = initFn_ + 0.5 * delta2_;
    delta3_ = ts_ * functionDot(gain_, pole_, initFn3_, fnT);
    initFn4_ = initFn_ + 0.5 * delta3_;
    delta4_ = ts_ * functionDot(gain_, pole_, initFn4_, fnT);

    // Solution
    yt_ = initFn_ + 1 / 6. * (delta1_ + 2. * delta2_ + 2. * delta3_ + delta4_);
    initFn_ = yt_;

    return yt_;
  }

  Eigen::VectorXf getEulerIntegral(const Eigen::VectorXf& fnT) {
    delta1_ = ts_ * functionDot(gain_, pole_, initFn_, fnT);

    // Solution
    yt_ = initFn_ + delta1_;
    initFn_ = yt_;

    return yt_;
  }

  void setGain(float _gain) { gain_ = _gain; }

  void setPole(float _pole) { pole_ = _pole; }

  void setSampleTime(float t) { ts_ = t; }
};