#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "iam_dual_arm_control/tools/Utils.hpp"
#include "sg_filter.h"
#include <deque>
#include <vector>

#define NB_ROBOTS 2// Number of robots
#define NB_FT_SENSOR_SAMPLES                                                                                           \
  50// Number of force torque sensors' samples used for initial calibration (compute the offsets)
#define MOVING_FORCE_WINDOW_SIZE 10// Window's size used to average the force data and detect peristent contact

class RobotVariable {

private:
  int nbJoints_[NB_ROBOTS];
  float normalForceAverage_[NB_ROBOTS];// Average normal force measured through the force windows [N]
  float normalForce_[NB_ROBOTS];       // Normal force to the surface [N]
  Eigen::Vector3f gravity_;
  float toolMass_[NB_ROBOTS];                           // Tool mass [kg]
  float toolOffsetFromEE_[NB_ROBOTS];                   // Tool offset along z axis of end effector [m]
  Eigen::Vector3f toolComPositionFromSensor_[NB_ROBOTS];// Tool CoM offset along z axis of end effector [m]
  int wrenchCount_[NB_ROBOTS];                          // Counter used to pre-process the force data
  std::deque<float>
      normalForceWindow_[NB_ROBOTS];// Moving window saving the robots measured normal force to the object's surface [N]
  bool wrenchBiasOK_[NB_ROBOTS];    // Check if computation of force/torque sensor bias is OK

  Eigen::Vector3f x_[NB_ROBOTS];
  Eigen::Vector4f q_[NB_ROBOTS];
  Eigen::Matrix3f wRb_[NB_ROBOTS];// Orientation matrix (3x3)
  Eigen::Vector3f v_[NB_ROBOTS];
  Eigen::Vector3f w_[NB_ROBOTS];
  Eigen::Vector4f qd_[NB_ROBOTS];

  Vector6f wrench_[NB_ROBOTS];    // Wrench [N and Nm] (6x1)
  Vector6f wrenchBias_[NB_ROBOTS];// Wrench bias [N and Nm] (6x1)

  Eigen::Matrix4f wHRb_[NB_ROBOTS];        // Homogenenous transform of robots base frame (4x4)
  Eigen::Matrix4f rbHEEStandby_[NB_ROBOTS];// Homogenenous transform of EE standby poses relatve to robots base (4x4)
  Eigen::Vector3f xrbStandby_[NB_ROBOTS];  // quaternion orientation of EE standby poses relatve to robots base (3x1)
  Eigen::Vector4f qrbStandby_[NB_ROBOTS];  // quaternion orientation of EE standby poses relatve to robots base (4x1)

  std::unique_ptr<SGF::SavitzkyGolayFilter> sgfDdqFilteredLeft_;
  std::unique_ptr<SGF::SavitzkyGolayFilter> sgfDdqFilteredRight_;

public:
  Eigen::Vector3f _aad[NB_ROBOTS];// desired axis angle

  Eigen::Vector3f _vd[NB_ROBOTS];
  Eigen::Vector3f _omegad[NB_ROBOTS];

  Vector6f _Vee[NB_ROBOTS];
  Vector6f _Vd_ee[NB_ROBOTS];     // desired velocity twist
  Matrix6f _tcp_W_EE[NB_ROBOTS];  // Velocity Twist transformation between the robot EE and the tool center point (tcp)
  Vector6f _VEE_oa[NB_ROBOTS];    // self collision (obstacle) avoidance
  Eigen::Vector3f _fxc[NB_ROBOTS];// Desired conservative parts of the nominal DS [m/s] (3x1)

  Vector6f _filteredWrench[NB_ROBOTS];// Filtered wrench [N and Nm] (6x1)

  Eigen::Matrix4f _w_H_ee[NB_ROBOTS];       // Homogenenous transform of the End-effectors poses (4x4)
  Eigen::Matrix4f _w_H_eeStandby[NB_ROBOTS];// Homogenenous transform of Standby pose of the End-effectors (4x4)

  Vector7f _joints_positions[NB_ROBOTS];
  Vector7f _joints_velocities[NB_ROBOTS];
  Vector7f _joints_accelerations[NB_ROBOTS];
  Vector7f _joints_torques[NB_ROBOTS];

  int getNbJoints(int robotID) { return nbJoints_[robotID]; }
  float getNormalForceAverage(int robotID) { return normalForceAverage_[robotID]; }
  float getNormalForce(int robotID) { return normalForce_[robotID]; }
  Eigen::Vector3f getX(int robotID) { return x_[robotID]; }
  Eigen::Vector4f getQ(int robotID) { return q_[robotID]; }
  Eigen::Vector4f getQdSpecific(int robotID) { return qd_[robotID]; }

  RobotVariable() {}
  ~RobotVariable(){};

  void init(int sgf_q[], float dt, Eigen::Vector3f gravity) {

    gravity_ = gravity;

    for (int k = 0; k < NB_ROBOTS; k++) {
      nbJoints_[k] = 7;
      x_[k].setConstant(0.0f);
      q_[k].setConstant(0.0f);
      wRb_[k].setIdentity();
      _w_H_ee[k].setConstant(0.0f);
      _w_H_eeStandby[k].setConstant(0.0f);
      wHRb_[k].setIdentity();
      rbHEEStandby_[k].setIdentity();
      xrbStandby_[k].setZero();
      qrbStandby_[k].setZero();

      // Desired values
      _Vd_ee[k].setZero();
      _Vee[k].setZero();
      _tcp_W_EE[k].setIdentity();
      v_[k].setZero();
      w_[k].setZero();
      _vd[k].setZero();
      _omegad[k].setZero();
      qd_[k].setZero();
      _aad[k].setZero();

      // Forces control variables
      _fxc[k].setZero();
      _filteredWrench[k].setZero();
      wrench_[k].setZero();
      wrenchBias_[k].setZero();
      normalForceAverage_[k] = 0.0f;
      wrenchCount_[k] = 0;
      normalForce_[k] = 0.0f;

      // Joint variables
      _joints_positions[k].setZero();
      _joints_velocities[k].setZero();
      _joints_accelerations[k].setZero();
      _joints_torques[k].setZero();

      _VEE_oa[k].setZero();
    }

    // Filtered variable (SG)
    sgfDdqFilteredLeft_ = std::make_unique<SGF::SavitzkyGolayFilter>(sgf_q[0],//dim
                                                                     sgf_q[1],//order
                                                                     sgf_q[2],//window length
                                                                     dt);
    sgfDdqFilteredRight_ = std::make_unique<SGF::SavitzkyGolayFilter>(sgf_q[0],//dim
                                                                      sgf_q[1],//order
                                                                      sgf_q[2],//window length
                                                                      dt);
  }

  void get_StandbyHmgTransformInBase() {
    _w_H_eeStandby[0] = Utils<float>::pose2HomoMx(xrbStandby_[0], qrbStandby_[0]);//
    _w_H_eeStandby[1] = Utils<float>::pose2HomoMx(xrbStandby_[1], qrbStandby_[1]);//
  }

  void get_StandbyHmgTransformInWorld() {
    _w_H_eeStandby[0] = wHRb_[0] * Utils<float>::pose2HomoMx(xrbStandby_[0], qrbStandby_[0]);//
    _w_H_eeStandby[1] = wHRb_[1] * Utils<float>::pose2HomoMx(xrbStandby_[1], qrbStandby_[1]);//
  }

  void get_EndEffectorHmgTransform() {
    _w_H_ee[0] = Utils<float>::pose2HomoMx(x_[0], q_[0]);// WITH EE pose wrt. the world
    _w_H_ee[1] = Utils<float>::pose2HomoMx(x_[1], q_[1]);// WITH EE pose wrt. the world
  }

  void get_desired_lin_task_velocity(float applyVelo, float nu_Wr0) {
    _vd[0] = applyVelo * _vd[0] + nu_Wr0 * _fxc[0];
    _vd[1] = applyVelo * _vd[1] + nu_Wr0 * _fxc[1];
  }

  void get_robotBaseFrameInWorld(Eigen::Vector3f xB, Eigen::Vector4f q, int k) {
    wHRb_[k].block(0, 3, 3, 1) = xB;
    wHRb_[k].block(0, 0, 3, 3) = Utils<float>::quaternionToRotationMatrix(q);
  }

  void update_EndEffectorPosesInWorld(Eigen::Vector3f xB, Eigen::Vector4f q, int k) {
    // update positions and Orientations of the EEs
    x_[k] = xB;
    q_[k] = q;

    // update EE positions with tool offset
    wRb_[k] = Utils<float>::quaternionToRotationMatrix(q_[k]);
    x_[k] = x_[k] + toolOffsetFromEE_[k] * wRb_[k].col(2);

    // update velocity Twist transformation from EE to tcp
    Eigen::Vector3f tcp = toolOffsetFromEE_[k] * wRb_[k].col(2);
    Eigen::Matrix3f skew_Mx_tcp;
    skew_Mx_tcp << 0.0f, -tcp(2), tcp(1), tcp(2), 0.0f, -tcp(0), -tcp(1), tcp(0), 0.0f;
    _tcp_W_EE[k].block(0, 3, 3, 3) = skew_Mx_tcp;
  }

  void update_EndEffectorVelocity(Eigen::Vector3f vE, Eigen::Vector3f wE, int k) {
    v_[k] = vE;
    w_[k] = wE;
    _Vee[k].head(3) = v_[k];
    _Vee[k].tail(3) = w_[k];
    _Vee[k] = _tcp_W_EE[k] * _Vee[k];
  }

  void update_EndEffectorWrench(Eigen::Matrix<float, 6, 1> raw,
                                Eigen::Vector3f normalObj[],
                                float filteredForceGain,
                                bool wrenchBiasOK[],
                                int k) {
    //
    if (!wrenchBiasOK[k]) {
      Eigen::Vector3f loadForce = wRb_[k].transpose() * toolMass_[k] * gravity_;
      wrenchBias_[k].segment(0, 3) -= loadForce;
      wrenchBias_[k].segment(3, 3) -= toolComPositionFromSensor_[k].cross(loadForce);
      wrenchBias_[k] += raw;
      wrenchCount_[k]++;

      if (wrenchCount_[k] == NB_FT_SENSOR_SAMPLES) {
        wrenchBias_[k] /= NB_FT_SENSOR_SAMPLES;
        wrenchBiasOK[k] = true;
        // std::cerr << "[robot]: Bias " << k << ": " <<wrenchBias_[k].transpose() << std::endl;
      }
    }

    if (wrenchBiasOK[k]) {
      wrench_[k] = raw - wrenchBias_[k];
      Eigen::Vector3f loadForce = wRb_[k].transpose() * toolMass_[k] * gravity_;
      wrench_[k].segment(0, 3) -= loadForce;
      wrench_[k].segment(3, 3) -= toolComPositionFromSensor_[k].cross(loadForce);
      wrench_[k].head(3) = wRb_[k] * wrench_[k].head(3);
      wrench_[k].tail(3) = wRb_[k] * wrench_[k].tail(3);
      _filteredWrench[k] = filteredForceGain * _filteredWrench[k] + (1.0f - filteredForceGain) * wrench_[k];
      //
      normalForce_[k] = fabs((_filteredWrench[k].segment(0, 3)).dot(normalObj[k]));
    }
  }

  void get_estimated_joint_accelerations(int k) {
    //
    SGF::Vec temp_acc(nbJoints_[k]);
    if (k == 0) {
      sgfDdqFilteredLeft_->AddData(_joints_velocities[k]);
      sgfDdqFilteredLeft_->GetOutput(1, temp_acc);
    } else {
      sgfDdqFilteredRight_->AddData(_joints_velocities[k]);
      sgfDdqFilteredRight_->GetOutput(1, temp_acc);
    }
    _joints_accelerations[k] = temp_acc.cast<float>();
  }

  void get_estimated_AverageNormalForce() {
    //
    for (int k = 0; k < NB_ROBOTS; k++) {
      if (normalForceWindow_[k].size() < MOVING_FORCE_WINDOW_SIZE) {
        normalForceWindow_[k].push_back(normalForce_[k]);
        normalForceAverage_[k] = 0.0f;
      } else {
        normalForceWindow_[k].pop_front();
        normalForceWindow_[k].push_back(normalForce_[k]);
        normalForceAverage_[k] = 0.0f;
        for (int m = 0; m < MOVING_FORCE_WINDOW_SIZE; m++) { normalForceAverage_[k] += normalForceWindow_[k][m]; }
        normalForceAverage_[k] /= MOVING_FORCE_WINDOW_SIZE;
      }
    }
  }

  void set_init_parameters(float toolMass_param[],
                           float toolOffsetFromEE_param[],
                           Eigen::Vector3f toolComPositionFromSensor_param[],
                           Eigen::Vector3f xrbStandby_param[],
                           Eigen::Vector4f qrbStandby_param[]) {
    //
    memcpy(toolMass_, &toolMass_param[0], NB_ROBOTS * sizeof *toolMass_param);
    memcpy(toolOffsetFromEE_, &toolOffsetFromEE_param[0], NB_ROBOTS * sizeof *toolOffsetFromEE_param);
    memcpy(toolComPositionFromSensor_,
           &toolComPositionFromSensor_param[0],
           NB_ROBOTS * sizeof *toolComPositionFromSensor_param);
    memcpy(xrbStandby_, &xrbStandby_param[0], NB_ROBOTS * sizeof *xrbStandby_param);
    memcpy(qrbStandby_, &qrbStandby_param[0], NB_ROBOTS * sizeof *qrbStandby_param);

    // get stanby transformation of the EEs wrt. the dual-robot Base frame
    this->get_StandbyHmgTransformInBase();
  }
};
