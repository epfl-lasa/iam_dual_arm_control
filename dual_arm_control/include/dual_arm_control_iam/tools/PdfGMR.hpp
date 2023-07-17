//|
//|    Copyright (C) 2021-2023 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors: Michael Bombile (maintainer)
//|
//|    email:   michael.bombile@epfl.ch/micbombile@gmail.com
//|
//|    Other contributors:
//|             Elise Jeandupeux (elise.jeandupeux@epfl.ch)
//|
//|    website: lasa.epfl.ch
//|
//|    This file is part of iam_dual_arm_control.
//|    This work was supported by the European Community's Horizon 2020 Research and Innovation
//|    programme (call: H2020-ICT-09-2019-2020, RIA), grant agreement 871899 Impact-Aware Manipulation.
//|
//|    iam_dual_arm_control is free software: you can redistribute it and/or modify  it under the terms
//|    of the GNU General Public License as published by  the Free Software Foundation,
//|    either version 3 of the License, or  (at your option) any later version.
//|
//|    iam_dual_arm_control is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|

#pragma once

#include "eigen3/Eigen/Core"

// #include "dual_arm_control_iam/DataLogging.hpp"

using namespace std;
class PdfGMR {
private:
  Eigen::VectorXf priorGMMToss_;
  Eigen::MatrixXf meanGMMToss_;
  Eigen::MatrixXf covMxGMMToss_;

  // DataLogging datalog_;

public:
  PdfGMR(){};
  ~PdfGMR(){};

  bool init(std::string fileGMM[]) {
    // datalog_.loadGMMParam(fileGMM, priorGMMToss_, meanGMMToss_, covMxGMMToss_);
    return true;
  }

  float gaussPDF(Eigen::VectorXf data, Eigen::VectorXf mu, Eigen::MatrixXf sigma) {

    Eigen::VectorXf deltaX = data - mu;
    float muX = deltaX.transpose() * pseudoInverse(sigma) * deltaX;

    return exp(-0.5f * muX) / std::sqrt(std::pow(2 * M_PI, data.rows()) * (std::fabs(sigma.determinant()) + 1e-30));
  }

  bool getGMRIO(Eigen::VectorXf prior,
                Eigen::MatrixXf meanGMM,
                Eigen::MatrixXf covMx,
                Eigen::VectorXf input,
                Eigen::VectorXf& output,
                Eigen::MatrixXf& expectedInputCov,
                Eigen::MatrixXf& expectedOutputCov) {

    // Extract the number of Gaussian
    int nStates = prior.rows();
    // Extract dimension of q_star and xi_star
    int nI = input.rows();
    int nO = meanGMM.rows() - nI;

    output.resize(nO);
    output.setZero();

    // Expected Input covariance q
    expectedInputCov.resize(nI, nI);
    expectedInputCov.setZero();
    // Expected Output covariance xi
    expectedOutputCov.resize(nO, nO);
    expectedOutputCov.setZero();

    Eigen::MatrixXf covInputK(nI, nI);
    Eigen::MatrixXf covOutputK(nO, nO);
    Eigen::MatrixXf covInOutK(nI, nO);
    Eigen::MatrixXf covOutInK(nO, nI);

    // |  nI | nIO |
    // |-----------|
    // | nOI | nO  |

    // Computation of h
    Eigen::VectorXf h = Eigen::VectorXf::Zero(nStates);
    Eigen::VectorXf normalH = Eigen::VectorXf::Zero(nStates);

    int dim = nI + nO;

    for (int k = 0; k < nStates; k++) {
      covInputK = covMx.block(k * dim, 0, nI, nI);

      h(k) = prior(k) * this->gaussPDF(input, meanGMM.col(k).head(nI), covInputK);
    }

    normalH = h / h.sum();

    Eigen::MatrixXf covK(dim, dim);

    for (int k = 0; k < nStates; k++) {

      covK = covMx.block(k * dim, 0, dim, dim);

      covInputK = covK.block(0, 0, nI, nI);
      covInOutK = covK.block(0, nI, nI, nO);
      covOutInK = covK.block(nI, 0, nO, nI);
      covOutputK = covK.block(nI, nI, nO, nO);

      // Posterior of q
      Eigen::VectorXf dInMu = input - meanGMM.col(k).head(nI);

      Eigen::MatrixXf sigmaOutInInverseSigmaIn = covOutInK * pseudoInverse(covInputK);

      output += normalH(k) * (meanGMM.col(k).tail(nO) + sigmaOutInInverseSigmaIn * dInMu);
      // Associated covaraince of Posterior q
      expectedOutputCov += normalH(k) * normalH(k) * (covOutputK - sigmaOutInInverseSigmaIn * covInOutK);
      expectedInputCov += normalH(k) * normalH(k) * (covInputK);
    }

    return true;
  }

  bool computeGMROutputs(Eigen::VectorXf input,
                         Eigen::VectorXf& output,
                         Eigen::MatrixXf& expectedInputCov,
                         Eigen::MatrixXf& expectedOutputCov) {

    this->getGMRIO(this->priorGMMToss_,
                   this->meanGMMToss_,
                   this->covMxGMMToss_,
                   input,
                   output,
                   expectedInputCov,
                   expectedOutputCov);
    return true;
  }

  // Compute the pseudo inverse of a matrix
  template<typename _Matrix_Type_>
  _Matrix_Type_ pseudoInverse(const _Matrix_Type_& a, double epsilon = std::numeric_limits<double>::epsilon()) {

    Eigen::JacobiSVD<_Matrix_Type_> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);

    double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);

    return svd.matrixV()
        * (svd.singularValues().array().abs() > tolerance)
              .select(svd.singularValues().array().inverse(), 0)
              .matrix()
              .asDiagonal()
        * svd.matrixU().adjoint();
  }
};