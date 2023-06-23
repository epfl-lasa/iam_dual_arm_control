/** Class ObjectToGrasp

*/

#pragma once

#ifndef OBJECT_TO_GRASP_H
#define OBJECT_TO_GRASP_H

#include "Utils.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "sg_filter.h"
#include <deque>
#include <vector>

#define NB_ROBOTS 2// Number of robots
#define NB_FT_SENSOR_SAMPLES                                                                                           \
  50// Number of force torque sensors' samples used for initial calibration (compute the offsets)
#define MOVING_FORCE_WINDOW_SIZE 10// Window's size used to average the force data and detect peristent contact
#define NB_OBJECTS 3               // Number of objects

typedef Eigen::Matrix<float, 7, 1> Vector7f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

class object_to_grasp {

public:
  // object
  float _objectMass;
  Eigen::Vector3f _objectDim;// Object dimensions [m] (3x1)
  Eigen::Vector3f _xo;
  Eigen::Vector4f _qo;
  Eigen::Vector3f _xDo;
  Eigen::Vector4f _qDo;
  Eigen::Matrix4f _w_H_o;
  Eigen::Matrix4f _w_H_Do;
  Eigen::Vector3f _xoC;// Measured object center position [m] (3x1)
  Eigen::Vector3f _xoD;// Measured object dimension vector [m] (3x1)
  Eigen::Vector3f _xgp_o[NB_ROBOTS];
  Eigen::Vector4f _qgp_o[NB_ROBOTS];
  Eigen::Matrix4f _w_H_gp[NB_ROBOTS];
  Eigen::Matrix4f _w_H_Dgp[NB_ROBOTS];
  Eigen::Vector3f _vo;
  Eigen::Vector3f _wo;
  Eigen::Vector3f _x_pickup;
  // Vector6f 		_Vo;
  // Vector6f 		_Vd_o;   													// desired object velocity (toss)
  // Vector6f  		_desired_object_wrench;
  Eigen::Vector3f _n[NB_ROBOTS];// Normal vector to surface object for each robot (3x1)
  Vector6f _V_gpo[NB_ROBOTS];

  std::unique_ptr<SGF::SavitzkyGolayFilter> _xo_filtered;
  std::unique_ptr<SGF::SavitzkyGolayFilter> _qo_filtered;
  // KF_3DVeloFromPosEstimator 								_xo_KF_filtered; //
  // KF_3DVeloFromPosEstimator 								_wo_KF_filtered; //

  object_to_grasp(){};
  ~object_to_grasp(){};

  void init_object(int sgf_p[], int sgf_o[], float dt, Eigen::Matrix3f o_R_gpl, Eigen::Matrix3f o_R_gpr) {
    // object
    _vo.setZero();
    _wo.setZero();
    _xo.setZero();
    _xDo.setZero();
    // _Vd_o.setZero();
    _xoC.setZero();
    _xoD.setZero();
    _x_pickup.setZero();

    _qo << 1.0f, 0.0f, 0.0f, 0.0f;
    _qDo << 1.0f, 0.0f, 0.0f, 0.0f;
    _w_H_o = Utils<float>::pose2HomoMx(_xo, _qo);
    _w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo);
    _qgp_o[0] = Utils<float>::rotationMatrixToQuaternion(o_R_gpl);//
    _qgp_o[1] = Utils<float>::rotationMatrixToQuaternion(o_R_gpr);//

    // normal to contact surfaces
    _n[0] = o_R_gpl.col(2);
    _n[1] = o_R_gpr.col(2);
    _V_gpo[0].setZero();
    _V_gpo[1].setZero();

    //
    _xo_filtered = std::make_unique<SGF::SavitzkyGolayFilter>(sgf_p[0], sgf_p[1], sgf_p[2], dt);//(3,3,6,_dt);
    _qo_filtered = std::make_unique<SGF::SavitzkyGolayFilter>(sgf_o[0],
                                                              sgf_o[1],
                                                              sgf_o[2],
                                                              dt);//(4,3,10,_dt); dim, order, win_l, dt

    // //
    // _xo_KF_filtered.init(_dt, Eigen::Vector2f(0.004, 0.1), 0.004, _xo);
    // _xo_KF_filtered.update(_xo);

    this->get_desiredHmgTransform();
  }

  void get_HmgTransform() { _w_H_o = Utils<float>::pose2HomoMx(_xo, _qo); }

  void get_desiredHmgTransform() { _w_H_Do = Utils<float>::pose2HomoMx(_xDo, _qDo); }

  void get_estimated_state() {
    // filtered object position
    SGF::Vec temp(3), temp_o(4);
    _xo_filtered->AddData(_xo);
    _xo_filtered->GetOutput(0, temp);
    _xo = temp;
    _xo_filtered->GetOutput(1, temp);
    _vo = temp;
    //
    _qo_filtered->AddData(_qo);
    _qo_filtered->GetOutput(0, temp_o);
    _qo = temp_o;

    // normalizing the quaternion
    _qo.normalize();
    //
    if (_qo.norm() <= 1e-8) { _qo = Eigen::Vector4f(1.0, 0.0, 0.0, 0.0); }
    //
    // ===========================================================
    _qo_filtered->GetOutput(1, temp_o);
    Eigen::Vector4f qo_dot = temp_o;
    //
    Eigen::MatrixXf wQ_map(3, 4);
    wQ_map << -_qo(1), -_qo(0), -_qo(3), _qo(2), -_qo(2), _qo(3), _qo(0), -_qo(1), -_qo(3), -_qo(2), _qo(1), _qo(0);
    _wo = 0.0 * wQ_map * qo_dot;
  }

  void get_grasp_point_HTransform() {
    _w_H_gp[0] = _w_H_o * Utils<float>::pose2HomoMx(_xgp_o[0], _qgp_o[0]);
    _w_H_gp[1] = _w_H_o * Utils<float>::pose2HomoMx(_xgp_o[1], _qgp_o[1]);
  }

  void get_grasp_point_desiredHTransform() {
    _w_H_Dgp[0] = _w_H_Do * Utils<float>::pose2HomoMx(_xgp_o[0], _qgp_o[0]);
    _w_H_Dgp[1] = _w_H_Do * Utils<float>::pose2HomoMx(_xgp_o[1], _qgp_o[1]);
  }

  void get_grasp_point_desiredRotation() {
    _w_H_Dgp[0].block(0, 0, 3, 3) =
        _w_H_Do.block(0, 0, 3, 3) * Utils<float>::pose2HomoMx(_xgp_o[0], _qgp_o[0]).block(0, 0, 3, 3);
    _w_H_Dgp[1].block(0, 0, 3, 3) =
        _w_H_Do.block(0, 0, 3, 3) * Utils<float>::pose2HomoMx(_xgp_o[1], _qgp_o[1]).block(0, 0, 3, 3);
  }

  void update_grasp_normals() {
    _n[0] = _w_H_gp[0].block(0, 0, 3, 3).col(2);
    _n[1] = _w_H_gp[1].block(0, 0, 3, 3).col(2);
  }

  void get_grasp_point_velocity() {
    //velocity of grasp points on the object
    for (int i = 0; i < NB_ROBOTS; i++) {
      Eigen::Vector3f t = _w_H_o.block<3, 3>(0, 0) * _xgp_o[i];
      Eigen::Matrix3f skew_Mx_gpo;
      skew_Mx_gpo << 0.0f, -t(2), t(1), t(2), 0.0f, -t(0), -t(1), t(0), 0.0f;
      // velocity
      _V_gpo[i].head(3) = _vo - 0 * skew_Mx_gpo * _wo;
      _V_gpo[i].tail(3) = 0 * _wo;
      //
      _V_gpo[i].head(3) *= 0.0f;
      _V_gpo[i].tail(3) *= 0.0f;
    }
  }
};

#endif// OBJECT_TO_GRASP_H