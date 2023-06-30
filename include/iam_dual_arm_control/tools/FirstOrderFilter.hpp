#pragma once

#include "Eigen/Eigen"

class FirstOrderFilter {
  double Ts;

  // Eigen::VectorXf init_fn;
  Eigen::VectorXf init_fn2;
  Eigen::VectorXf init_fn3;
  Eigen::VectorXf init_fn4;
  Eigen::VectorXf delta1;
  Eigen::VectorXf delta2;
  Eigen::VectorXf delta3;
  Eigen::VectorXf delta4;
  Eigen::VectorXf y_t;

public:
  float pole;
  float gain;
  Eigen::VectorXf init_fn;

  FirstOrderFilter() {}
  //
  void InitializeFilter(float T, float gn, float pl, Eigen::VectorXf init_fn_val) {
    Ts = T;
    gain = gn;
    pole = pl;

    init_fn.resize(init_fn_val.rows(), init_fn_val.cols());
    init_fn2.resize(init_fn_val.rows(), init_fn_val.cols());
    init_fn3.resize(init_fn_val.rows(), init_fn_val.cols());
    init_fn4.resize(init_fn_val.rows(), init_fn_val.cols());

    delta1.resize(init_fn_val.rows(), init_fn_val.cols());
    delta2.resize(init_fn_val.rows(), init_fn_val.cols());
    delta3.resize(init_fn_val.rows(), init_fn_val.cols());
    delta4.resize(init_fn_val.rows(), init_fn_val.cols());

    y_t.resize(init_fn_val.rows(), init_fn_val.cols());
    init_fn = init_fn_val;
  }

  ~FirstOrderFilter() {}

  Eigen::VectorXf function_dot(float gn, float pl, const Eigen::VectorXf& init_fn_val, const Eigen::VectorXf& fn_t) {
    return -pl * init_fn_val + gn * fn_t;
  }

  // compute the integral of first order differential eq. using RK4
  Eigen::VectorXf getRK4Integral(const Eigen::VectorXf& fn_t) {
    delta1 = Ts * function_dot(gain, pole, init_fn, fn_t);
    init_fn2 = init_fn + 0.5 * delta1;
    delta2 = Ts * function_dot(gain, pole, init_fn2, fn_t);
    init_fn3 = init_fn + 0.5 * delta2;
    delta3 = Ts * function_dot(gain, pole, init_fn3, fn_t);
    init_fn4 = init_fn + 0.5 * delta3;
    delta4 = Ts * function_dot(gain, pole, init_fn4, fn_t);

    // solution
    y_t = init_fn + 1 / 6. * (delta1 + 2. * delta2 + 2. * delta3 + delta4);
    init_fn = y_t;

    return y_t;
  }

  Eigen::VectorXf getEulerIntegral(const Eigen::VectorXf& fn_t) {
    delta1 = Ts * function_dot(gain, pole, init_fn, fn_t);
    // solution
    y_t = init_fn + delta1;
    init_fn = y_t;

    return y_t;
  }

  void setGain(float _gain) { gain = _gain; }

  void setPole(float _pole) { pole = _pole; }

  void setSampleTime(float T) { Ts = T; }
};