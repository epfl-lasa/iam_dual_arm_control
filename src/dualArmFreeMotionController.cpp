
#include "iam_dual_arm_control/dualArmFreeMotionController.h"
#include "iam_dual_arm_control/Utils.hpp"


float computeCouplingFactor(Eigen::Vector3f ep_, float alpha_, float beta_, float gamma_, bool secondOrder)
{
  float t_cpl_ = 1.0f/(alpha_*ep_.norm()+1e-15f);         
  float cpl_   = 0.0f;
  t_cpl_ = pow(t_cpl_,gamma_);
  if(secondOrder) cpl_   = 1.0f - exp(-t_cpl_/beta_) *(1.0f + t_cpl_/beta_);  // 2nd order critically damped
  else            cpl_   = 1.0f - exp(-t_cpl_/beta_);                         // 1st order increase

  return cpl_;
}

dualArmFreeMotionController::dualArmFreeMotionController()
{
  _error_abs.setZero();
  _error_rel.setZero();
  _error_obj.setZero();
  _V_abs.setZero();
  _V_rel.setZero();
  _V_obj.setZero();
  gain_p_abs.setZero();
  gain_o_abs.setZero();
  gain_p_rel.setZero();
  gain_o_rel.setZero();
  Omega_object_d_.setZero();

  _coord_abs2 = 0.0f;
  _cpl_rel    = 0.0f;
  _cp_ap      = 0.0f;
  //
  _Tbi = Eigen::MatrixXf::Identity(6,6);
  _Tbi.topLeftCorner(3,3)     = 0.5f* Eigen::MatrixXf::Identity(3,3);
  _Tbi.topRightCorner(3,3)    = 0.5f* Eigen::MatrixXf::Identity(3,3);
  _Tbi.bottomLeftCorner(3,3)  =      -Eigen::MatrixXf::Identity(3,3);
  _Tbi.bottomRightCorner(3,3) =       Eigen::MatrixXf::Identity(3,3);
  //

  a_proximity_    = 0.0f;
  a_proximity_    = 0.0f;
  a_normal_       = 0.0f;
  a_tangent_      = 0.0f;
  a_retract_      = 0.0f;
  a_release_      = 0.0f;
  release_flag_   = false;
  //
  rho_          = 0.12;
  range_norm_   = 0.05;
  range_tang_   = 0.015;

  sw_proxim_    = 100.0;
  sw_norm_      = 150.0;
  sw_tang_      = 200.0;
  //
  a_normal_Do_  = 0.0f;
  _desVreach    = 1.0f;
  // _refVreach    = 0.0f;
  _refVreach[LEFT]  = 0.0f;
  _refVreach[RIGHT] = 0.0f;
  _refVtoss_EE  = 0.0f;
  _modulated_reaching = true;
  _isNorm_impact_vel  = false;
  _height_via_point = 0.25f;

  //
  _sw_EE_obsAv  = 100.0f;
  _min_dist_EE  = 0.09f;
  //
  Eigen::Vector2f P1 = Eigen::Vector2f(0.f, 0.085f);
  Eigen::Vector2f P2 = Eigen::Vector2f(0.f,-0.085f);
  Eigen::Vector2f P3 = Eigen::Vector2f(0.04f, 0.f);

  Eigen::Matrix3f Mdelta, Mx0, My0;
  Mdelta <<                     P1(0), P1(1), 1,                       P2(0), P2(1), 1,                        P3(0), P3(1), 1;
  Mx0    << pow(P1(0),2)+pow(P1(1),2), P1(1), 1,   pow(P2(0),2)+pow(P2(1),2), P2(1), 1,    pow(P3(0),2)+pow(P3(1),2), P3(1), 1;
  My0    << pow(P1(0),2)+pow(P1(1),2), P1(0), 1,   pow(P2(0),2)+pow(P2(1),2), P2(0), 1,    pow(P3(0),2)+pow(P3(1),2), P3(0), 1;

  float delta = 2* Mdelta.determinant();
  Eigen::Vector2f center_ssphere_xz = Eigen::Vector2f(1/delta * Mx0.determinant(), -1/delta * My0.determinant());
  _safe_radius= center_ssphere_xz.norm();
  _integral_Vee_d[LEFT].setZero();
  _integral_Vee_d[RIGHT].setZero();

  _Twist_vo.setZero();
  _w_H_vo.setIdentity();
  _w_H_vgp[0].setIdentity();
  _w_H_vgp[1].setIdentity();
  _dt = 0.005;
  _objectDim << 0.20f, 0.20f, 0.20f;
  _go2object = 0.0f;
  _alpha_obs[LEFT]  = 0.0f;
  _alpha_obs[RIGHT] = 0.0f;
  _smoothcount = -100;

}

dualArmFreeMotionController::~dualArmFreeMotionController(){}

// publishing of the reference trajectories
bool dualArmFreeMotionController::init(Eigen::Matrix4f w_H_eeStandby[], Matrix6f gain_abs_, Matrix6f gain_rel_)
{
  //
  memcpy(_w_H_eeStandby, &w_H_eeStandby[0], NB_ROBOTS * sizeof * w_H_eeStandby);
  //
  reachable_p = 1.0f;
  // _v_max       = 0.7f;
  // _w_max       = 2.0f;
  // _v_max = 2.0f;     // velocity limits
  // _w_max = 4.0f;     // velocity limits
  _v_max      = 1.0f;
  _w_max      = 3.0f;

  gain_p_abs = gain_abs_.topLeftCorner(3,3);
  gain_o_abs = gain_abs_.bottomRightCorner(3,3);
  gain_p_rel = gain_rel_.topLeftCorner(3,3);
  gain_o_rel = gain_rel_.bottomRightCorner(3,3);
  //
  qdPrev[LEFT]  << 1.0f, 0.0f, 0.0f, 0.0f;
  qdPrev[RIGHT] << 1.0f, 0.0f, 0.0f, 0.0f;
  _cpl_grasp = 0.0f;
  //
}
void dualArmFreeMotionController::computeCoordinatedMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d)
{
  //
  float coord_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 1.0f, true);
  // Computation of desired orientation
  this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  Eigen::Matrix4f w_H_dgp_l = w_H_gp[LEFT];  
  w_H_dgp_l.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]);
  Eigen::Matrix4f w_H_dgp_r = w_H_gp[RIGHT]; 
  w_H_dgp_r.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);

  // get the task transformations
  Eigen::Matrix4f w_H_ar, lr_H_rr;           // absolute and relative EE poses
  Eigen::Matrix4f w_H_ap, lp_H_rp;           // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_ar_stb, lr_H_rr_stb;   // absolute and relative EE standby poses
  Eigen::Matrix4f lp_H_rp_pgrasp;            // relative pregrasp EE pose
  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(w_H_ee[LEFT], w_H_ee[RIGHT], w_H_ar, lr_H_rr);    // EE
  // Utils<float>::getBimanualTransforms(w_H_gp[LEFT], w_H_gp[RIGHT], w_H_ap, lp_H_rp);      // object's grasp points
  Utils<float>::getBimanualTransforms(w_H_dgp_l, w_H_dgp_r, w_H_ap, lp_H_rp);      // object's grasp points
  Utils<float>::getBimanualTransforms(this->_w_H_eeStandby[LEFT], this->_w_H_eeStandby[RIGHT], w_H_ar_stb, lr_H_rr_stb); // standby arms
  //
  lp_H_rp_pgrasp       = lp_H_rp;
  lp_H_rp_pgrasp(1, 3) = lp_H_rp(1,3)/fabs(lp_H_rp(1,3)) * (fabs(lp_H_rp(1,3)) + 0.30f);

  // =======================================
  // Absolute velocity of the End-effectors
  // =======================================
  Eigen::Vector3f d_p_abs = reachable_p *w_H_ap.block<3,1>(0,3) + (1.0f-reachable_p)*w_H_ar_stb.block<3,1>(0,3);
  _error_abs.head(3)      = w_H_ar.block<3,1>(0,3) - d_p_abs;
  
  // Coupling the orientation with the position error
  float cpl_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 1.0f, false);

  Eigen::Matrix3f d_R_abs = reachable_p *w_H_ap.block<3,3>(0,0) + (1.0f-reachable_p)*w_H_ar_stb.block<3,3>(0,0);
  Eigen::Matrix4f w_H_ar_t = w_H_ar;
  w_H_ar_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(cpl_abs, w_H_ar.block<3,3>(0,0), d_R_abs); //desired
  // relative transformation
  Eigen::Matrix4f d_H_c_abs = w_H_ar_t.inverse() * w_H_ar;
  // orientation error
  _error_abs.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_abs).tail(3);
  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_abs = Utils<float>::getMuThetaJacobian(d_H_c_abs.block<3,3>(0,0)) * w_H_ar.block<3,3>(0,0).transpose();

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  _V_abs.head(3) = -gain_p_abs * _error_abs.head(3);
  _V_abs.tail(3) = -jacMuTheta_abs.inverse() * gain_o_abs * _error_abs.tail(3);

  // =====================================
  // Relative velocity of the hands
  // =====================================
  // float coord_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.01f, 1.0f, true);
  // Coupling the orientation with the position error
  Eigen::Matrix3f d_R_rel = reachable_p* (coord_abs*lp_H_rp.block<3,3>(0,0) + (1.0f-coord_abs)*lp_H_rp_pgrasp.block<3,3>(0,0)) 
                          + (1.0f- reachable_p)*lr_H_rr_stb.block<3,3>(0,0);

  Eigen::Matrix4f lr_H_rr_t = lr_H_rr;
  lr_H_rr_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(coord_abs, lr_H_rr.block<3,3>(0,0), d_R_rel); //desired
  // relative transformation
  // Eigen::Matrix4f d_H_c_rel = lr_H_rr_t.inverse() * lr_H_rr;  // expressed in the left hand frame 

  // ///////////////////////////////////////////////////////////////////////////////////////////
  // // Computation of desired orientation
  // this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);

  // ///////////////////////////////////////////////////////////////////////////////////////////
  // relative transformation
  lr_H_rr_t.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]).transpose() * Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);
  Eigen::Matrix4f d_H_c_rel = lr_H_rr_t.inverse() * lr_H_rr;  // expressed in the left hand frame 
  
  // orientation error
  _error_rel.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_rel).tail(3);
  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_rel = Utils<float>::getMuThetaJacobian(d_H_c_rel.block<3,3>(0,0)) * w_H_ee[RIGHT].block<3,3>(0,0).transpose(); // wrt. the world

  // ///////////////////////////////////////////////////////////////////////////////////////
  float cpl_rel    = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.08f, 1.0f, true);  // 50.0f, 0.05f, 2.8f  0.5

  Eigen::Vector3f o_error_pos_abs = w_H_o.block<3,3>(0,0).transpose() * _error_abs.head(3);
  Eigen::Vector3f o_error_pos_abs_paral = Eigen::Vector3f(o_error_pos_abs(0), 0.0f, o_error_pos_abs(2));
  float cp_ap = computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.12f, 1.0f, true);  // 50.0f, 0.05f, 2.8f  (0.04f) (0.02f)


  // position error accounting for the reachability of the target
  // Eigen::Vector3f d_p_rel = reachable_p *(cpl_rel*cp_ap* lp_H_rp.block<3,1>(0,3) + (1.0f-cpl_rel*cp_ap) *lp_H_rp_pgrasp.block<3,1>(0,3)) + (1.0f-reachable_p) * lr_H_rr_stb.block<3,1>(0,3); // TBC 
  Eigen::Vector3f d_p_rel = cpl_rel *(cp_ap * lp_H_rp.block<3,1>(0,3) + (1.0f-cp_ap) *lp_H_rp_pgrasp.block<3,1>(0,3)) + (1.0f-cpl_rel) * lr_H_rr_stb.block<3,1>(0,3); // TBC 

  _error_rel.head(3) = lr_H_rr.block<3,1>(0,3) - d_p_rel;  // 

  // computing the velocity
  _V_rel.head(3) = -gain_p_rel * _error_rel.head(3);
  _V_rel.tail(3) = -jacMuTheta_rel.inverse() * gain_o_rel * _error_rel.tail(3);

  // ========================================
  // Computation of individual EE motion
  // ========================================
  // velocity
  float a_bi = 0.5f;
  float b_bi = 1.0f;
  Utils<float>::getBimanualTwistDistribution(a_bi, b_bi, _V_abs, _V_rel, Vd_ee[LEFT], Vd_ee[RIGHT]);

  Vd_ee[LEFT]  = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[LEFT]);
  Vd_ee[RIGHT] = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[RIGHT]);

  // Vd_ee[LEFT].head(3)  = Vd_ee[LEFT].head(3) + Vd_ee[LEFT].head(3)/Vd_ee[LEFT].head(3).norm() * cp_ap * 0.2;
  // Vd_ee[RIGHT].head(3) = Vd_ee[RIGHT].head(3) + Vd_ee[RIGHT].head(3)/Vd_ee[RIGHT].head(3).norm() * cp_ap * 0.2;

  // std::cout << "[dual_arm_control]: CCCCCCCCCCC coord_abs: \t" <<  coord_abs << std::endl;
  // std::cout << "[dual_arm_control]: CCCCCCCCCCC TANH: \t" <<  1.0f-std::tanh(3.0f*_error_rel.head(3).norm()) << std::endl;
  // std::cout << "[dual_arm_control]: CCCCCCCCCCC cpl_rel: \t" <<  cpl_rel << std::endl;
  // std::cout << "[dual_arm_control]: Absolute Error: \t" <<  _V_abs.transpose() << std::endl;
  // std::cout << "[dual_arm_control]: Absolute Velo: \t" <<  _V_abs.transpose() << std::endl;
  // std::cout << "[dual_arm_control]: Relative Velo: \t" <<  _V_rel.transpose() << std::endl;

  // // Computation of desired orientation
  // // this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  // // this->computeDesiredOrientation(1.0f-std::tanh(3.0f*_error_rel.head(3).norm()), w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  // this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
}


void dualArmFreeMotionController::computeConstrainedMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d)
{
  // Computation of desired orientation
  this->computeDesiredOrientation(1.0f, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  Eigen::Matrix4f w_H_dgp_l = w_H_gp[LEFT];  
  w_H_dgp_l.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]);
  Eigen::Matrix4f w_H_dgp_r = w_H_gp[RIGHT]; 
  w_H_dgp_r.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);

  // get the task transformations
  Eigen::Matrix4f w_H_ar, lr_H_rr;           // absolute and relative EE poses
  Eigen::Matrix4f w_H_ap, lp_H_rp;           // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_ar_stb, lr_H_rr_stb;   // absolute and relative EE standby poses
  Eigen::Matrix4f lp_H_rp_pgrasp;            // relative pregrasp EE pose
  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(w_H_ee[LEFT], w_H_ee[RIGHT], w_H_ar, lr_H_rr);    // EE
  // Utils<float>::getBimanualTransforms(w_H_gp[LEFT], w_H_gp[RIGHT], w_H_ap, lp_H_rp);      // object's grasp points
  Utils<float>::getBimanualTransforms(w_H_dgp_l, w_H_dgp_r, w_H_ap, lp_H_rp);      // object's grasp points
  Utils<float>::getBimanualTransforms(this->_w_H_eeStandby[LEFT], this->_w_H_eeStandby[RIGHT], w_H_ar_stb, lr_H_rr_stb); // standby arms
  //
  lp_H_rp_pgrasp      = lp_H_rp;
  lp_H_rp_pgrasp(1, 3) *= 1.4; // * lp_H_rp(1, 3);

  // =======================================
  // Absolute velocity of the End-effectors
  // =======================================
  Eigen::Vector3f d_p_abs = reachable_p *w_H_ap.block<3,1>(0,3) + (1.0f-reachable_p)*w_H_ar_stb.block<3,1>(0,3);
  _error_abs.head(3)      = w_H_ar.block<3,1>(0,3) - d_p_abs;

  std::cout << "[dual_arm_control]: Absolute Error: \n" <<  _V_abs.transpose() << std::endl;
  // std::cout << "[dual_arm_control]: Absolute Gain: \n" <<  this->gain_p_abs << std::endl;

  // Coupling the orientation with the position error
  float cpl_abs = 1.0f; //computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 1.0f, false);

  Eigen::Matrix3f d_R_abs = reachable_p *w_H_ap.block<3,3>(0,0) + (1.0f-reachable_p)*w_H_ar_stb.block<3,3>(0,0);
  Eigen::Matrix4f w_H_ar_t = w_H_ar;
  w_H_ar_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(cpl_abs, w_H_ar.block<3,3>(0,0), d_R_abs); //desired
  // relative transformation
  Eigen::Matrix4f d_H_c_abs = w_H_ar_t.inverse() * w_H_ar;
  // orientation error
  _error_abs.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_abs).tail(3);
  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_abs = Utils<float>::getMuThetaJacobian(d_H_c_abs.block<3,3>(0,0)) * w_H_ar.block<3,3>(0,0).transpose();

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  _V_abs.head(3) = -4.0f*gain_p_abs * _error_abs.head(3);
  _V_abs.tail(3) = -1.0f*jacMuTheta_abs.inverse() * gain_o_abs * _error_abs.tail(3);

  std::cout << "[dual_arm_control]: Absolute Velo: \n" <<  _V_abs.transpose() << std::endl;

  // =====================================
  // Relative velocity of the hands
  // =====================================
  float coord_abs = 1.0f; //computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 1.0f, true);
  // Coupling the orientation with the position error
  Eigen::Matrix3f d_R_rel = reachable_p* (coord_abs*lp_H_rp.block<3,3>(0,0) + (1.0f-coord_abs)*lp_H_rp_pgrasp.block<3,3>(0,0)) 
                          + (1.0f- reachable_p)*lr_H_rr_stb.block<3,3>(0,0);

  Eigen::Matrix4f lr_H_rr_t = lr_H_rr;
  lr_H_rr_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(coord_abs, lr_H_rr.block<3,3>(0,0), d_R_rel); //desired
  // // relative transformation
  // Eigen::Matrix4f d_H_c_rel = lr_H_rr_t.inverse() * lr_H_rr;  // expressed in the left hand frame 

  // ///////////////////////////////////////////////////////////////////////////////////////////
  // Computation of desired orientation
  // this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);

  // ///////////////////////////////////////////////////////////////////////////////////////////
  // relative transformation
  lr_H_rr_t.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]).transpose() * Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);
  Eigen::Matrix4f d_H_c_rel = lr_H_rr_t.inverse() * lr_H_rr;  // expressed in the left hand frame 
  // orientation error
  _error_rel.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_rel).tail(3);
  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_rel = Utils<float>::getMuThetaJacobian(d_H_c_rel.block<3,3>(0,0)) * w_H_ee[RIGHT].block<3,3>(0,0).transpose(); // wrt. the world

  // ///////////////////////////////////////////////////////////////////////////////////////
  float cpl_rel    = 1.0f; //computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 2.8f, true);  // 50.0f, 0.05f, 2.8f

  // position error accounting for the reachability of the target
  Eigen::Vector3f d_p_rel = reachable_p *(cpl_rel * lp_H_rp.block<3,1>(0,3) + (1.0f-cpl_rel) *lp_H_rp_pgrasp.block<3,1>(0,3))
                          + (1.0f-reachable_p) * lr_H_rr_stb.block<3,1>(0,3); // TBC  
  _error_rel.head(3) = lr_H_rr.block<3,1>(0,3) - d_p_rel;

  // computing the velocity
  _V_rel.head(3) = -4.0f*gain_p_rel * _error_rel.head(3);
  _V_rel.tail(3) = -4.0f*jacMuTheta_rel.inverse() * gain_o_rel * _error_rel.tail(3);

  std::cout << "[dual_arm_control]: Relative Velo: \n" <<  _V_rel.transpose() << std::endl;

  // ========================================
  // Computation of individual EE motion
  // ========================================
  // velocity
  float a_bi = 0.5f;
  float b_bi = 1.0f;
  Utils<float>::getBimanualTwistDistribution(a_bi, b_bi, _V_abs, _V_rel, Vd_ee[LEFT], Vd_ee[RIGHT]);

  // Vd_ee[LEFT]  = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[LEFT]);
  // Vd_ee[RIGHT] = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[RIGHT]);

  // // Computation of desired orientation
  // // this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  // this->computeDesiredOrientation(1.0f-std::tanh(3.0f*_error_rel.head(3).norm()), w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
}

void dualArmFreeMotionController::computeAsyncMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d)
{
  //
  for(int k=0; k<NB_ROBOTS; k++)
  {
    Vector6f error_ee;  error_ee.setZero();
    // Eigen::Vector3f d_p_ee   = reachable_p *w_H_gp[k].block<3,1>(0,3) + (1.0f-reachable_p)*w_H_ee[k].block<3,1>(0,3);
    // Eigen::Matrix3f d_R_ee   = reachable_p *w_H_gp[k].block<3,3>(0,0) + (1.0f-reachable_p)*w_H_ee[k].block<3,3>(0,0);
    Eigen::Vector3f d_p_ee   = reachable_p *w_H_gp[k].block<3,1>(0,3) + (1.0f-reachable_p)*this->_w_H_eeStandby[k].block<3,1>(0,3);
    Eigen::Matrix3f d_R_ee   = reachable_p *w_H_gp[k].block<3,3>(0,0) + (1.0f-reachable_p)*this->_w_H_eeStandby[k].block<3,3>(0,0); 
    Eigen::Matrix4f w_H_ee_t = w_H_ee[k];
    w_H_ee_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(1.0f, w_H_ee[k].block<3,3>(0,0), d_R_ee); //desired
    // relative transformation between desired and current frame
    Eigen::Matrix4f d_H_c_ee = w_H_ee_t.inverse() * w_H_ee[k];
    error_ee.head(3)         = w_H_ee[k].block<3,1>(0,3) - d_p_ee;
    error_ee.tail(3)         = Utils<float>::getPoseErrorCur2Des(d_H_c_ee).tail(3);
    // 3D Orientation Jacobian 
    Eigen::Matrix3f jacMuTheta_ee = Utils<float>::getMuThetaJacobian(d_H_c_ee.block<3,3>(0,0)) * w_H_ee[k].block<3,3>(0,0).transpose();
    // ---------------------------------
    // computing of desired ee velocity
    // ---------------------------------
    Vd_ee[k].head(3) = -gain_p_abs * error_ee.head(3);
    Vd_ee[k].tail(3) = -jacMuTheta_ee.inverse() * gain_o_abs * error_ee.tail(3);
    Vd_ee[k]         = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[k]);
  }
  // Computation of desired orientation
  this->computeDesiredOrientation(0.5f, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
}

void dualArmFreeMotionController::computeDesiredOrientation(float weight, Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d)
{
  
  if(isOrient3d)
  {
    for(int k = 0; k<NB_ROBOTS; k++)
    {
      qd[k]  = Utils<float>::getSlerpInterpolation(weight, w_H_ee[k].block(0,0, 3,3), w_H_gp[k].block(0,0, 3,3));
    }
  }
  else
  {
    for(int k = 0; k < NB_ROBOTS; k++)
    {
      Eigen::Vector3f ref;
      ref = w_H_gp[k].block<3,1>(0,2);
      ref.normalize();

      // Compute rotation error between current orientation and plane orientation using Rodrigues' law
      Eigen::Vector3f u;

      // u = (_wRb[k].col(2)).cross(ref);
      // float c = (_wRb[k].col(2)).transpose()*ref;  
      u = (w_H_ee[k].block<3,3>(0,0).col(2)).cross(ref);
      float s = u.norm();
      u /= s;
      float c = (w_H_ee[k].block<3,3>(0,0).col(2)).transpose()*ref;  
      
      Eigen::Matrix3f K;
      K << Utils<float>::getSkewSymmetricMatrix(u);

      Eigen::Matrix3f Re;
      if(fabs(s)< FLT_EPSILON)
      {
        Re = Eigen::Matrix3f::Identity();
      }
      else
      {
        Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
      }
      
      // Convert rotation error into axis angle representation
      Eigen::Vector3f omega;
      float angle;
      Eigen::Vector4f qtemp = Utils<float>::rotationMatrixToQuaternion(Re);
      Eigen::Vector4f q_    = Utils<float>::rotationMatrixToQuaternion(w_H_ee[k].block<3,3>(0,0));
      Utils<float>::quaternionToAxisAngle(qtemp,omega,angle);

      // Compute final quaternion on plane
      Eigen::Vector4f qf = Utils<float>::quaternionProduct(qtemp,q_);
      // Eigen::Vector4f qf = Utils<float>::quaternionProduct(q_,qtemp);

      // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the object surface
      // _qd[k] = Utils<float>::slerpQuaternion(q_[k],qf,1.0f-std::tanh(3.0f*_eD)); // _error_rel.head(3)
      qd[k] = Utils<float>::slerpQuaternion(q_,qf,1.0f-std::tanh(3.0f*_error_rel.head(3).norm())); // _error_rel.head(3)
      // qd[k] = Utils<float>::slerpQuaternion(q_,qf,1.0f); // _error_rel.head(3)

      if(qd[k].dot(qdPrev[k])<0.0f)
      {
        qd[k] *=-1.0f;
      }

      qdPrev[k] = qd[k];
    }
  }


}


void dualArmFreeMotionController::computeReleaseAndRetractMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d)
{
  // Computation of desired orientation
  this->computeDesiredOrientation(1.0f, w_H_ee, _w_H_eeStandby, w_H_o, qd, isOrient3d);
  Eigen::Matrix4f w_H_dgp_l = w_H_gp[LEFT];  
  w_H_dgp_l.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]);
  Eigen::Matrix4f w_H_dgp_r = w_H_gp[RIGHT]; 
  w_H_dgp_r.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);

  // get the task transformations
  Eigen::Matrix4f w_H_ar, lr_H_rr;           // absolute and relative EE poses
  Eigen::Matrix4f w_H_ap, lp_H_rp;           // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_ar_stb, lr_H_rr_stb;   // absolute and relative EE standby poses
  Eigen::Matrix4f lp_H_rp_pgrasp;            // relative pregrasp EE pose
  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(w_H_ee[LEFT], w_H_ee[RIGHT], w_H_ar, lr_H_rr);    // EE
  // Utils<float>::getBimanualTransforms(w_H_gp[LEFT], w_H_gp[RIGHT], w_H_ap, lp_H_rp);      // object's grasp points
  Utils<float>::getBimanualTransforms(w_H_dgp_l, w_H_dgp_r, w_H_ap, lp_H_rp);      // object's grasp points
  Utils<float>::getBimanualTransforms(this->_w_H_eeStandby[LEFT], this->_w_H_eeStandby[RIGHT], w_H_ar_stb, lr_H_rr_stb); // standby arms
  //
  lp_H_rp_pgrasp      = lp_H_rp;
  lp_H_rp_pgrasp(1, 3) *= 1.5; // * lp_H_rp(1, 3);


  // =====================================
  // Relative velocity of the hands
  // =====================================
  // float coord_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 1.0f, true);
  // Coupling the orientation with the position error
  Eigen::Matrix3f d_R_rel = lr_H_rr_stb.block<3,3>(0,0);

  Eigen::Matrix4f lr_H_rr_t = lr_H_rr;
  lr_H_rr_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(1.0f, lr_H_rr.block<3,3>(0,0), d_R_rel); //desired
  // relative transformation
  // Eigen::Matrix4f d_H_c_rel = lr_H_rr_t.inverse() * lr_H_rr;  // expressed in the left hand frame 

  // ///////////////////////////////////////////////////////////////////////////////////////////
  // Computation of desired orientation
  // this->computeDesiredOrientation(1.0f, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  // this->computeDesiredOrientation(1.0f, w_H_ee, _w_H_eeStandby, w_H_o, qd, isOrient3d);

  // ///////////////////////////////////////////////////////////////////////////////////////////
  // relative transformation
  lr_H_rr_t.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]).transpose() * Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);
  Eigen::Matrix4f d_H_c_rel = lr_H_rr_t.inverse() * lr_H_rr;  // expressed in the left hand frame 

  
  // orientation error
  _error_rel.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_rel).tail(3);
  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_rel = Utils<float>::getMuThetaJacobian(d_H_c_rel.block<3,3>(0,0)) * w_H_ee[RIGHT].block<3,3>(0,0).transpose(); // wrt. the world

  // ///////////////////////////////////////////////////////////////////////////////////////
  // float cpl_rel    = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 2.8f, true);  // 50.0f, 0.05f, 2.8f

  // float cpl_rel    = computeCouplingFactor(_error_rel.tail(3), 50.0f, 0.03f, 1.0f, true);  // 50.0f, 0.05f, 2.8f


  // position error accounting for the reachability of the target
  Eigen::Vector3f d_p_rel = lr_H_rr_stb.block<3,1>(0,3); // TBC  
  _error_rel.head(3) = lr_H_rr.block<3,1>(0,3) - d_p_rel;

  // computing the velocity
  _V_rel.head(3) = -4.0f*gain_p_rel * _error_rel.head(3);
  _V_rel.tail(3) = -jacMuTheta_rel.inverse() * gain_o_rel * _error_rel.tail(3);

  std::cout << "[dual_arm_control]: Relative Velo: \t" <<  _V_rel.transpose() << std::endl;

  // =======================================
  // Absolute velocity of the End-effectors
  // =======================================
  float cpl_rel    = computeCouplingFactor(_error_rel.head(3), 50.0f, 0.01f, 1.0f, true);  // 50.0f, 0.05f, 2.8f

  Eigen::Vector3f d_p_abs = (1.f - cpl_rel) *w_H_ar.block<3,1>(0,3) + cpl_rel*w_H_ar_stb.block<3,1>(0,3);
  _error_abs.head(3)      = w_H_ar.block<3,1>(0,3) - d_p_abs;
  std::cout << "[dual_arm_control]: Absolute Error: \t" <<  _V_abs.transpose() << std::endl;

  // Coupling the orientation with the position error
  float cpl_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 1.0f, false);  // 0.2f
  cpl_abs = 1.0f;

  Eigen::Matrix3f d_R_abs = (1.f - cpl_abs) *w_H_ar.block<3,3>(0,0) + cpl_abs*w_H_ar_stb.block<3,3>(0,0);
  Eigen::Matrix4f w_H_ar_t = w_H_ar;
  w_H_ar_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(cpl_abs, w_H_ar.block<3,3>(0,0), d_R_abs); //desired
  // relative transformation
  Eigen::Matrix4f d_H_c_abs = w_H_ar_t.inverse() * w_H_ar;
  // orientation error
  _error_abs.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_abs).tail(3);
  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_abs = Utils<float>::getMuThetaJacobian(d_H_c_abs.block<3,3>(0,0)) * w_H_ar.block<3,3>(0,0).transpose();

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  _V_abs.head(3) = -gain_p_abs * _error_abs.head(3);
  _V_abs.tail(3) = -jacMuTheta_abs.inverse() * gain_o_abs * _error_abs.tail(3);

  std::cout << "[dual_arm_control]: Absolute Velo: \t" <<  _V_abs.transpose() << std::endl;

  // ========================================
  // Computation of individual EE motion
  // ========================================
  // velocity
  float a_bi = 0.5f;
  float b_bi = 1.0f;
  Utils<float>::getBimanualTwistDistribution(a_bi, b_bi, _V_abs, _V_rel, Vd_ee[LEFT], Vd_ee[RIGHT]);

  for(int k=0; k<NB_ROBOTS; k++){
    Eigen::Matrix3f d_R_ee   = Utils<float>::quaternionToRotationMatrix(qd[k]);
    Eigen::Matrix4f w_H_ee_t = w_H_ee[k];
          w_H_ee_t.block<3,3>(0,0) = d_R_ee; //desired
    Eigen::Matrix4f d_H_c_ee = w_H_ee_t.inverse() * w_H_ee[k];
    Eigen::Vector3f error_ee_o    = Utils<float>::getPoseErrorCur2Des(d_H_c_ee).tail(3);
    Eigen::Matrix3f jacMuTheta_ee = Utils<float>::getMuThetaJacobian(d_H_c_ee.block<3,3>(0,0)) * w_H_ee[k].block<3,3>(0,0).transpose();
    Vd_ee[k].tail(3) = -4.0f*jacMuTheta_ee.inverse() * gain_o_abs * error_ee_o;
  } 

  Vd_ee[LEFT]  = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[LEFT]);
  Vd_ee[RIGHT] = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[RIGHT]);

  std::cout << "[dual_arm_control]: CCCCCCCCCCC cpl_abs: \t" <<  cpl_abs << std::endl;
  std::cout << "[dual_arm_control]: CCCCCCCCCCC TANH: \t" <<  1.0f-std::tanh(3.0f*_error_rel.head(3).norm()) << std::endl;
  std::cout << "[dual_arm_control]: CCCCCCCCCCC cpl_rel: \t" <<  cpl_rel << std::endl;

  // // Computation of desired orientation
  // // this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  // // this->computeDesiredOrientation(1.0f-std::tanh(3.0f*_error_rel.head(3).norm()), w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  // this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
}

void dualArmFreeMotionController::generatePlacingMotion(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, float via_height,
                                                        Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d)
{
  // Computation of desired orientation
  this->computeDesiredOrientation(1.0f, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  Eigen::Matrix4f w_H_dgp_l = w_H_gp[LEFT];  
  w_H_dgp_l.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]);
  Eigen::Matrix4f w_H_dgp_r = w_H_gp[RIGHT]; 
  w_H_dgp_r.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);
  // 
  Eigen::Matrix4f w_H_o_z, w_H_Do_z;   // current and desired object pose but with height of via plane
  Eigen::Matrix4f w_H_ap, lp_H_rp;     // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_ar, lr_H_rr;     // absolute and relative EE poses

  w_H_o_z       = w_H_o;
  w_H_Do_z      = w_H_Do;
  w_H_o_z(2,3)  = w_H_Do(2,3) + via_height;
  w_H_Do_z(2,3) = w_H_Do(2,3) + via_height;
  // Bimanual transformation
  // Utils<float>::getBimanualTransforms(w_H_gp[LEFT], w_H_gp[RIGHT], w_H_ap, lp_H_rp);    // object's grasp points
  Utils<float>::getBimanualTransforms(w_H_dgp_l, w_H_dgp_r, w_H_ap, lp_H_rp);      // object's grasp points
  Utils<float>::getBimanualTransforms(w_H_ee[LEFT], w_H_ee[RIGHT], w_H_ar, lr_H_rr);    // EE

  Eigen::Vector3f error_z  = Eigen::Vector3f(0.f, 0.f, w_H_o(2,3) - w_H_o_z(2,3));
  Eigen::Vector3f error_xy = Eigen::Vector3f(w_H_o(0,3)-w_H_Do_z(0,3), w_H_o(1,3)-w_H_Do_z(1,3), 0.0f); //w_H_o.block<2,1>(0,3) - w_H_Do_z.block<2,1>(0,3); 

  // float cpl_oz   = Utils<float>::computeCouplingFactor(error_z, 50.0f, 0.12f, 1.2f, true);
  float cpl_oz   = 1.0f-std::tanh(8.0f*error_z.norm());
  float cpl_Doxy = Utils<float>::computeCouplingFactor(error_xy, 50.0f, 0.10f, 1.0f, false);
  // ================================================================
  // Desired Object motion : Absolute velocity of the End-effectors
  // ================================================================
  float sat_cpl_z = ((cpl_oz + cpl_Doxy) <= 1.f) ? (cpl_oz + cpl_Doxy) : 1.f;
  float coord_pos = Utils<float>::computeCouplingFactor(error_xy, 50.0f, 0.02f, 1.0f, true);  //  Coupling the orientation function of planar position error
  Eigen::Matrix4f w_H_o_t   = w_H_o;
  w_H_o_t.block<3,3>(0,0)   = Utils<float>::getCombinedRotationMatrix(coord_pos, w_H_o.block<3,3>(0,0), w_H_Do.block<3,3>(0,0)); //desired
  // Relative pose of the current object pose relative to the desired one
  Eigen::Matrix4f d_H_c_obj = w_H_o_t.inverse() * w_H_o;                      // relative transformation
  Eigen::Vector3f d_pos_obj = sat_cpl_z * (cpl_Doxy * w_H_Do.block<3,1>(0,3) + (1.f - cpl_Doxy)*w_H_Do_z.block<3,1>(0,3)) + (1.f-sat_cpl_z)*w_H_o_z.block<3,1>(0,3);
  Eigen::Matrix3f jacMuTheta_obj = Utils<float>::getMuThetaJacobian(d_H_c_obj.block<3,3>(0,0)) * w_H_o.block<3,3>(0,0).transpose();     // 3D Orientation Jacobian 
  // pose error
  _error_obj.head(3) = w_H_o.block<3,1>(0,3) - d_pos_obj;                     // position error
  _error_obj.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_obj).tail(3);  // orientation error
  //
  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  _V_obj.head(3) = -gain_p_abs * _error_obj.head(3);
  _V_obj.tail(3) = -jacMuTheta_obj.inverse() * gain_o_abs * _error_obj.tail(3);
  // Computation of desired orientation
  // this->computeDesiredOrientation(1.0f, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  
  // ================================================================
  // Compute relative hand velocity to maintain the grasp
  // ================================================================
  // relative velocity
  Eigen::Matrix3f d_R_rel   = lp_H_rp.block<3,3>(0,0);
  Eigen::Matrix4f lr_H_rr_t = lr_H_rr;
  // Computation of desired orientation
  // this->computeDesiredOrientation(1.0f, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  lr_H_rr_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(1.0f, lr_H_rr.block<3,3>(0,0), d_R_rel); //desired
  // lr_H_rr_t.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]).transpose() * Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);
  Eigen::Matrix4f d_H_c_rel      = lr_H_rr_t.inverse() * lr_H_rr;  // expressed in the left hand frame  
  Eigen::Matrix3f jacMuTheta_rel = Utils<float>::getMuThetaJacobian(d_H_c_rel.block<3,3>(0,0)) * w_H_ee[RIGHT].block<3,3>(0,0).transpose(); //   // 3D Orientation Jacobian wrt. the world
  Eigen::Vector3f d_pos_rel      = lp_H_rp.block<3,1>(0,3); // TBC  

  _error_rel.head(3) = lr_H_rr.block<3,1>(0,3) - d_pos_rel;
  _error_rel.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_rel).tail(3);      // orientation error
  // computing the velocity
  _V_rel.head(3) = -4.0f*gain_p_rel * _error_rel.head(3);
  _V_rel.tail(3) = -4.0f*jacMuTheta_rel.inverse() * gain_o_rel * _error_rel.tail(3);

  // ================================================================
  // compute the grasp matrix to distribute the motion
  // ================================================================
  Eigen::Matrix<float, 6, 12> GraspMx_obj_EE;
  GraspMx_obj_EE.block<6,6>(0,0).setIdentity();
  GraspMx_obj_EE.block<6,6>(0,6).setIdentity();
  //
  Eigen::Matrix3f skew_Mx_[NB_ROBOTS];
  for(int i=0; i<NB_ROBOTS; i++)
  {
    Eigen::Vector3f t = w_H_ee[i].block<3,1>(0,3) - w_H_o.block<3,1>(0,3);
    skew_Mx_[i] <<      0.0f,   -t(2),      t(1),
                        t(2),    0.0f,     -t(0),
                       -t(1),    t(0),      0.0f;             
  }
  GraspMx_obj_EE.block<3,3>(3,0) = -skew_Mx_[LEFT];    // left EE
  GraspMx_obj_EE.block<3,3>(3,6) = -skew_Mx_[RIGHT];   // right EE
  //
  // ===================================================================
  // Computation of individual EE motion
  // ===================================================================
  Vd_ee[LEFT]   = GraspMx_obj_EE.block<6,6>(0,0).transpose() * _V_obj - 0.5f* _V_rel;
  Vd_ee[RIGHT]  = GraspMx_obj_EE.block<6,6>(0,6).transpose() * _V_obj + 0.5f* _V_rel;

  // applying velocity
  // ========================================
  Vd_ee[LEFT]  = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[LEFT]);
  Vd_ee[RIGHT] = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[RIGHT]);

  std::cout << "[dual_arm_control]: CCCCCCCCCCC cpl_oz: \t" <<  cpl_oz << std::endl;
  std::cout << "[dual_arm_control]: CCCCCCCCCCC cpl_Doxy: \t" <<  cpl_Doxy << std::endl;
}





void dualArmFreeMotionController::computeCoordinatedMotion2(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d)
{
  //
  float coord_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 1.0f, true);
  // Computation of desired orientation
  this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  Eigen::Matrix4f w_H_dgp_l = w_H_gp[LEFT];  
  w_H_dgp_l.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]);
  Eigen::Matrix4f w_H_dgp_r = w_H_gp[RIGHT]; 
  w_H_dgp_r.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);

  // get the task transformations
  Eigen::Matrix4f w_H_ar, lr_H_rr;           // absolute and relative EE poses
  Eigen::Matrix4f w_H_ap, lp_H_rp;           // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_ar_stb, lr_H_rr_stb;   // absolute and relative EE standby poses
  Eigen::Matrix4f lp_H_rp_pgrasp;            // relative pregrasp EE pose
  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(w_H_ee[LEFT], w_H_ee[RIGHT], w_H_ar, lr_H_rr);    // EE
  // Utils<float>::getBimanualTransforms(w_H_gp[LEFT], w_H_gp[RIGHT], w_H_ap, lp_H_rp);      // object's grasp points
  Utils<float>::getBimanualTransforms(w_H_dgp_l, w_H_dgp_r, w_H_ap, lp_H_rp);      // object's grasp points
  Utils<float>::getBimanualTransforms(this->_w_H_eeStandby[LEFT], this->_w_H_eeStandby[RIGHT], w_H_ar_stb, lr_H_rr_stb); // standby arms
  //
  lp_H_rp_pgrasp       = lp_H_rp;
  lp_H_rp_pgrasp(1, 3) = lp_H_rp(1,3)/fabs(lp_H_rp(1,3)) * (fabs(lp_H_rp(1,3)) + 0.30f);

  // =======================================
  // Absolute velocity of the End-effectors
  // =======================================
  Eigen::Vector3f d_p_abs = reachable_p *w_H_ap.block<3,1>(0,3) + (1.0f-reachable_p)*w_H_ar_stb.block<3,1>(0,3);
  _error_abs.head(3)      = w_H_ar.block<3,1>(0,3) - d_p_abs;
  
  // Coupling the orientation with the position error
  float cpl_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 1.0f, false);

  Eigen::Matrix3f d_R_abs = reachable_p *w_H_ap.block<3,3>(0,0) + (1.0f-reachable_p)*w_H_ar_stb.block<3,3>(0,0);
  Eigen::Matrix4f w_H_ar_t = w_H_ar;
  w_H_ar_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(cpl_abs, w_H_ar.block<3,3>(0,0), d_R_abs); //desired
  // relative transformation
  Eigen::Matrix4f d_H_c_abs = w_H_ar_t.inverse() * w_H_ar;
  // orientation error
  _error_abs.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_abs).tail(3);
  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_abs = Utils<float>::getMuThetaJacobian(d_H_c_abs.block<3,3>(0,0)) * w_H_ar.block<3,3>(0,0).transpose();

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  _V_abs.head(3) = -3.0f* gain_p_abs * _error_abs.head(3); // -3.0
  _V_abs.tail(3) = -3.0f* jacMuTheta_abs.inverse() * gain_o_abs * _error_abs.tail(3);

  // =====================================
  // Relative velocity of the hands
  // =====================================
  // float coord_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.01f, 1.0f, true);
  // Coupling the orientation with the position error
  Eigen::Matrix3f d_R_rel = reachable_p* (coord_abs*lp_H_rp.block<3,3>(0,0) + (1.0f-coord_abs)*lp_H_rp_pgrasp.block<3,3>(0,0)) 
                          + (1.0f- reachable_p)*lr_H_rr_stb.block<3,3>(0,0);

  Eigen::Matrix4f lr_H_rr_t = lr_H_rr;
  lr_H_rr_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(coord_abs, lr_H_rr.block<3,3>(0,0), d_R_rel); //desired
  // relative transformation
  // Eigen::Matrix4f d_H_c_rel = lr_H_rr_t.inverse() * lr_H_rr;  // expressed in the left hand frame 

  // ///////////////////////////////////////////////////////////////////////////////////////////
  // Computation of desired orientation
  // this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  // this->computeDesiredOrientation(1.0f, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);

  // ///////////////////////////////////////////////////////////////////////////////////////////
  // relative transformation
  lr_H_rr_t.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]).transpose() * Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);
  Eigen::Matrix4f d_H_c_rel = lr_H_rr_t.inverse() * lr_H_rr;  // expressed in the left hand frame 
  
  // orientation error
  _error_rel.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_rel).tail(3);
  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_rel = Utils<float>::getMuThetaJacobian(d_H_c_rel.block<3,3>(0,0)) * w_H_ee[RIGHT].block<3,3>(0,0).transpose(); // wrt. the world

  // ///////////////////////////////////////////////////////////////////////////////////////
  float cpl_rel    = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.08f, 1.0f, true);  // 50.0f, 0.05f, 2.8f  0.5

  Eigen::Vector3f o_error_pos_abs = w_H_o.block<3,3>(0,0).transpose() * _error_abs.head(3);
  Eigen::Vector3f o_error_pos_abs_paral = Eigen::Vector3f(o_error_pos_abs(0), 0.0f, o_error_pos_abs(2));
  float cp_ap = computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.06f, 1.0f, true);  // 50.0f, 0.12f, 1.0f  (0.04f) (0.02f)


  // position error accounting for the reachability of the target
  // Eigen::Vector3f d_p_rel = reachable_p *(cpl_rel*cp_ap* lp_H_rp.block<3,1>(0,3) + (1.0f-cpl_rel*cp_ap) *lp_H_rp_pgrasp.block<3,1>(0,3)) + (1.0f-reachable_p) * lr_H_rr_stb.block<3,1>(0,3); // TBC 
  Eigen::Vector3f d_p_rel = cpl_rel *(cp_ap * lp_H_rp.block<3,1>(0,3) + (1.0f-cp_ap) *lp_H_rp_pgrasp.block<3,1>(0,3)) + (1.0f-cpl_rel) * lr_H_rr_stb.block<3,1>(0,3); // TBC 

  _error_rel.head(3) = lr_H_rr.block<3,1>(0,3) - d_p_rel;  // 

  // computing the velocity
  _V_rel.head(3) = -5.0f* gain_p_rel * _error_rel.head(3);  // -3.0
  _V_rel.tail(3) = -3.0f* jacMuTheta_rel.inverse() * gain_o_rel * _error_rel.tail(3);  // -3.0

  // ========================================
  // Computation of individual EE motion
  // ========================================
  // velocity
  float a_bi = 0.5f;
  float b_bi = 1.0f;
  Utils<float>::getBimanualTwistDistribution(a_bi, b_bi, _V_abs, _V_rel, Vd_ee[LEFT], Vd_ee[RIGHT]);

  Eigen::Matrix4f d_H_c_l = w_H_dgp_l.inverse() * w_H_ee[LEFT];
  Eigen::Matrix4f d_H_c_r = w_H_dgp_r.inverse() * w_H_ee[RIGHT];

  // orientation error
  Eigen::Vector3f error_ori_l = Utils<float>::getPoseErrorCur2Des(d_H_c_l).tail(3);
  Eigen::Vector3f error_ori_r = Utils<float>::getPoseErrorCur2Des(d_H_c_r).tail(3);

  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_l = Utils<float>::getMuThetaJacobian(d_H_c_l.block<3,3>(0,0)) * w_H_ee[LEFT].block<3,3>(0,0).transpose(); // wrt. the world
  Eigen::Matrix3f jacMuTheta_r = Utils<float>::getMuThetaJacobian(d_H_c_r.block<3,3>(0,0)) * w_H_ee[RIGHT].block<3,3>(0,0).transpose(); // wrt. the world

  Vd_ee[LEFT].tail(3)  = -3.0f* jacMuTheta_l.inverse() * gain_o_rel * error_ori_l;
  Vd_ee[RIGHT].tail(3) = -3.0f* jacMuTheta_r.inverse() * gain_o_rel * error_ori_r;


  Vd_ee[LEFT]  = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[LEFT]);
  Vd_ee[RIGHT] = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[RIGHT]);

}

Vector6f dualArmFreeMotionController::generatePlacingMotion2(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, float via_height, Vector6f Vo)
{
  // 
  Eigen::Matrix4f w_H_o_z, w_H_Do_z;   // current and desired object pose but with height of via plane
  Eigen::Matrix4f w_H_ap, lp_H_rp;     // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_ar, lr_H_rr;     // absolute and relative EE poses

  w_H_o_z       = w_H_o;
  w_H_Do_z      = w_H_Do;
  w_H_o_z(2,3)  = w_H_Do(2,3) + 0.0*via_height;
  w_H_Do_z(2,3) = w_H_Do(2,3) + via_height;

  Eigen::Vector3f error_z  = Eigen::Vector3f(0.f, 0.f, w_H_o(2,3) - w_H_o_z(2,3));
  Eigen::Vector3f error_xy = Eigen::Vector3f(w_H_o(0,3)-w_H_Do_z(0,3), w_H_o(1,3)-w_H_Do_z(1,3), 0.0f); //w_H_o.block<2,1>(0,3) - w_H_Do_z.block<2,1>(0,3); 

  // float cpl_oz   = Utils<float>::computeCouplingFactor(error_z, 50.0f, 0.12f, 1.2f, true);
  // float cpl_oz   = 1.0f-std::tanh(8.0f*error_z.norm());
  // float cpl_Doxy = Utils<float>::computeCouplingFactor(error_xy, 50.0f, 0.10f, 1.0f, false);
  float cpl_oz   = 0.5f*(std::tanh(1.5f*this->sw_norm_  * (1.2f*this->range_norm_ - error_z.norm()))  + 1.0f );
  float cpl_Doxy = 0.5f*(std::tanh(1.5f*this->sw_norm_  * (2.0f*this->range_norm_ - error_xy.norm()))  + 1.0f );
  // ================================================================
  // Desired Object motion : Absolute velocity of the End-effectors
  // ================================================================
  float sat_cpl_z = ((cpl_oz + cpl_Doxy) <= 1.f) ? (cpl_oz + cpl_Doxy) : 1.f;
  float coord_pos = Utils<float>::computeCouplingFactor(error_xy, 50.0f, 0.02f, 1.0f, true);  //  Coupling the orientation function of planar position error
  Eigen::Matrix4f w_H_o_t   = w_H_o;
  w_H_o_t.block<3,3>(0,0)   = Utils<float>::getCombinedRotationMatrix(coord_pos, w_H_o.block<3,3>(0,0), w_H_Do.block<3,3>(0,0)); //desired
  // Relative pose of the current object pose relative to the desired one
  Eigen::Matrix4f d_H_c_obj = w_H_o_t.inverse() * w_H_o;                      // relative transformation
  Eigen::Vector3f d_pos_obj = sat_cpl_z * (cpl_Doxy * w_H_Do.block<3,1>(0,3) + (1.f - cpl_Doxy)*w_H_Do_z.block<3,1>(0,3)) + (1.f-sat_cpl_z)*w_H_o_z.block<3,1>(0,3);
  Eigen::Matrix3f jacMuTheta_obj = Utils<float>::getMuThetaJacobian(d_H_c_obj.block<3,3>(0,0)) * w_H_o.block<3,3>(0,0).transpose();     // 3D Orientation Jacobian 
  // pose error
  _error_obj.head(3) = w_H_o.block<3,1>(0,3) - d_pos_obj;                     // position error
  _error_obj.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_obj).tail(3);  // orientation error
  //
  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  _V_obj.head(3) = -3.0*gain_p_abs * _error_obj.head(3);
  _V_obj.tail(3) = -3.0*jacMuTheta_obj.inverse() * gain_o_abs * _error_obj.tail(3);
  //
  std::cout << "[dual_arm_control]: CCCCCCCCCCC cpl_oz: \t" <<  cpl_oz << std::endl;
  std::cout << "[dual_arm_control]: CCCCCCCCCCC cpl_Doxy: \t" <<  cpl_Doxy << std::endl;

  return _V_obj;
}

void dualArmFreeMotionController::set_virtual_object_frame(Eigen::Matrix4f w_H_vo){
  _w_H_vo = w_H_vo;
}

void dualArmFreeMotionController::computeCoordinatedMotion3(Eigen::Matrix4f w_H_ee[],  Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Vector6f Vo, Eigen::Vector3f _x_intercept, 
                                                            Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool isOrient3d){
  //
  float coord_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 1.0f, true);
  // Computation of desired orientation
  this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  Eigen::Matrix4f w_H_dgp_l = w_H_gp[LEFT];  
  w_H_dgp_l.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]);
  Eigen::Matrix4f w_H_dgp_r = w_H_gp[RIGHT]; 
  w_H_dgp_r.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);

  // get the task transformations
  Eigen::Matrix4f w_H_ar, lr_H_rr;               // absolute and relative EE poses
  Eigen::Matrix4f w_H_ap, lp_H_rp;               // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_avp, lp_H_rvp;             // absolute and relative virtual object's grasp points
  Eigen::Matrix4f w_H_ar_stb, lr_H_rr_stb;       // absolute and relative EE standby poses
  Eigen::Matrix4f lp_H_rp_pgrasp;                // relative pregrasp EE pose
  //
  //---------------------------------------
  //velocity of grasp points on the object
  //---------------------------------------
  Vector6f V_gpo[NB_ROBOTS];
  for(int i=0; i<NB_ROBOTS; i++)
  {
    // Eigen::Vector3f t = _w_H_o.block<3,3>(0,0) * _xgp_o[i];
    Eigen::Vector3f t = w_H_ee[i].block<3,1>(0,3) - w_H_o.block<3,1>(0,3);
    Eigen::Matrix3f skew_Mx_gpo;
    skew_Mx_gpo <<   0.0f,  -t(2),     t(1),
                     t(2),   0.0f,    -t(0),
                    -t(1),   t(0),     0.0f;             
    // velocity
    V_gpo[i].head(3) = Vo.head(3) - skew_Mx_gpo * Vo.tail(3);
    V_gpo[i].tail(3) = Vo.tail(3);
    //
  }
  //---------------------------------------
  _w_H_vo.block<3,3>(0,0) = w_H_o.block<3,3>(0,0);
  // Eigen::Vector3f pvo_star = (1.f - reachable_p)* _x_intercept + reachable_p* w_H_o.block<3,1>(0,3);           // _go2object
  Eigen::Vector3f pvo_star = (1.f - _go2object)* _x_intercept + _go2object* w_H_o.block<3,1>(0,3);                // 
  _Twist_vo.head(3) = -2.0f*gain_p_abs * (_w_H_vo.block<3,1>(0,3) - pvo_star) + _go2object * 0.0* Vo.head(3);     
  _Twist_vo.tail(3) = 0.0* Vo.tail(3);

  Utils<float>::UpdatePose_From_VelocityTwist(this->_dt, _Twist_vo, _w_H_vo);                                     // ////

  _w_H_vo = (1.f - _go2object)* _w_H_vo + _go2object* w_H_o;

  Eigen::Vector3f xvgp_o_d[NB_ROBOTS];

  bool object_between_ee = ((w_H_ee[LEFT](1,3) - (w_H_o(1,3)-_objectDim(1)/2.f) < 0.0f ) && (w_H_ee[RIGHT](1,3) - (w_H_o(1,3)+_objectDim(1)/2.f) > 0.0f ));
  Eigen::MatrixXf  p_rel_dot_vo = (w_H_ee[RIGHT].block<3,1>(0,3) - w_H_ee[LEFT].block<3,1>(0,3)).transpose() *Vo.head(3);

  if(!object_between_ee && (p_rel_dot_vo(0,0) < 0.0f)){       // object moving from right to left
    _alpha_obs[LEFT]  = 0.0f;
    _alpha_obs[RIGHT] = 1.0f;
  }
  else if(!object_between_ee && (p_rel_dot_vo(0,0) > 0.0f)){  // object moving from left to right
    _alpha_obs[LEFT]  = 1.0f;
    _alpha_obs[RIGHT] = 0.0f;
  }
  else{
    float tau = 0.20;
    _alpha_obs[LEFT]  =  (1.f-tau)*_alpha_obs[LEFT]  + tau * 0.0f;  // 0.0f;
    _alpha_obs[RIGHT] =  (1.f-tau)*_alpha_obs[RIGHT] + tau * 0.0f;  // 0.0f;
    // _smoothcount ++;
  }
  //
  Eigen::Matrix4f o_H_gp[NB_ROBOTS];
  for(int k=0; k<NB_ROBOTS; k++){
    o_H_gp[k]   = w_H_o.inverse() * w_H_gp[k];
    xvgp_o_d[k] = o_H_gp[k].block<3,1>(0,3)  + (1.f - _go2object) *_alpha_obs[k]  * Eigen::Vector3f(0.0f, 0.0f, _objectDim(2)/2.f + 0.09f);
    _w_H_vgp[k].block<3,3>(0,0) = _w_H_vo.block<3,3>(0,0) * o_H_gp[k].block<3,3>(0,0);
    _w_H_vgp[k].block<3,1>(0,3) = _w_H_vo.block<3,3>(0,0) * xvgp_o_d[k] + _w_H_vo.block<3,1>(0,3);
  }

  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(w_H_ee[LEFT], w_H_ee[RIGHT], w_H_ar, lr_H_rr);                                      // EE
  Utils<float>::getBimanualTransforms(_w_H_vgp[LEFT], _w_H_vgp[RIGHT], w_H_avp, lp_H_rvp);                                // EE
  // Utils<float>::getBimanualTransforms(w_H_gp[LEFT], w_H_gp[RIGHT], w_H_ap, lp_H_rp);                                   // object's grasp points
  Utils<float>::getBimanualTransforms(w_H_dgp_l, w_H_dgp_r, w_H_ap, lp_H_rp);                                             // object's grasp points
  Utils<float>::getBimanualTransforms(this->_w_H_eeStandby[LEFT], this->_w_H_eeStandby[RIGHT], w_H_ar_stb, lr_H_rr_stb);  // standby arms
  //
  // lp_H_rp_pgrasp       = lp_H_rp;
  // lp_H_rp_pgrasp(1, 3) = lp_H_rp(1,3)/fabs(lp_H_rp(1,3)) * (fabs(lp_H_rp(1,3)) + 0.30f);
  lp_H_rp_pgrasp       = lp_H_rvp;
  lp_H_rp_pgrasp(1, 3) = lp_H_rvp(1,3)/fabs(lp_H_rvp(1,3)) * (fabs(lp_H_rvp(1,3)) + 0.30f);

  // =======================================
  // Absolute velocity of the End-effectors
  // =======================================
  Eigen::Vector3f d_p_abs = reachable_p *w_H_avp.block<3,1>(0,3) + (1.0f-reachable_p)*w_H_ar_stb.block<3,1>(0,3);
  _error_abs.head(3)      = w_H_ar.block<3,1>(0,3) - d_p_abs;
  Eigen::Vector3f error_abs_ro = w_H_ar.block<3,1>(0,3) - w_H_o.block<3,1>(0,3);
  
  // Coupling the orientation with the position error
  float cpl_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 1.0f, false);

  Eigen::Matrix3f d_R_abs  = reachable_p *w_H_avp.block<3,3>(0,0) + (1.0f-reachable_p)*w_H_ar_stb.block<3,3>(0,0);
  Eigen::Matrix4f w_H_ar_t = w_H_ar;
  w_H_ar_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(cpl_abs, w_H_ar.block<3,3>(0,0), d_R_abs); //desired
  // relative transformation
  Eigen::Matrix4f d_H_c_abs = w_H_ar_t.inverse() * w_H_ar;
  // orientation error
  _error_abs.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_abs).tail(3);
  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_abs = Utils<float>::getMuThetaJacobian(d_H_c_abs.block<3,3>(0,0)) * w_H_ar.block<3,3>(0,0).transpose();

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  _V_abs.head(3) = -8.0f* gain_p_abs * _error_abs.head(3);
  // _V_abs.head(3) = -((1.f - _go2object)*0.5f + _go2object *1.0f)* gain_p_abs * _error_abs.head(3);
  _V_abs.tail(3) = -1.0f* jacMuTheta_abs.inverse() * gain_o_abs * _error_abs.tail(3);

  // =====================================
  // Relative velocity of the hands
  // =====================================
  // Coupling the orientation with the position error
  Eigen::Matrix3f d_R_rel = reachable_p* (coord_abs*lp_H_rvp.block<3,3>(0,0) + (1.0f-coord_abs)*lp_H_rp_pgrasp.block<3,3>(0,0)) 
                          + (1.0f- reachable_p)*lr_H_rr_stb.block<3,3>(0,0);

  Eigen::Matrix4f lr_H_rr_t = lr_H_rr;
  // ///////////////////////////////////////////////////////////////////////////////////////////
  // relative transformation
  lr_H_rr_t.block<3,3>(0,0) = Utils<float>::quaternionToRotationMatrix(qd[LEFT]).transpose() * Utils<float>::quaternionToRotationMatrix(qd[RIGHT]);
  Eigen::Matrix4f d_H_c_rel = lr_H_rr_t.inverse() * lr_H_rr;  // expressed in the left hand frame 
  
  // orientation error
  _error_rel.tail(3) = Utils<float>::getPoseErrorCur2Des(d_H_c_rel).tail(3);
  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_rel = Utils<float>::getMuThetaJacobian(d_H_c_rel.block<3,3>(0,0)) * w_H_ee[RIGHT].block<3,3>(0,0).transpose(); // wrt. the world

  // ///////////////////////////////////////////////////////////////////////////////////////
  float cpl_rel    = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.08f, 1.0f, true);  // 50.0f, 0.05f, 2.8f  0.5
  // float cpl_rel    = computeCouplingFactor(error_abs_ro, 50.0f, 0.08f, 1.0f, true);  // 50.0f, 0.05f, 2.8f  0.5

  // Eigen::Vector3f o_error_pos_abs = w_H_o.block<3,3>(0,0).transpose() * _error_abs.head(3);
  Eigen::Vector3f o_error_pos_abs = _w_H_vo.block<3,3>(0,0).transpose() * error_abs_ro; 
  Eigen::Vector3f o_error_pos_abs_paral = Eigen::Vector3f(o_error_pos_abs(0), 0.0f, o_error_pos_abs(2));
  // float cp_ap = computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.06f, 1.0f, true);  // 50.0f, 0.12f, 1.0f  (0.04f) (0.02f)
  float cp_ap = computeCouplingFactor(error_abs_ro, 60.0f, 0.15f, 1.0f, true);  // 50.0f, 0.12f, 1.0f  (0.04f) (0.02f)


  // position error accounting for the reachability of the target
  // Eigen::Vector3f d_p_rel = reachable_p *(cpl_rel*cp_ap* lp_H_rp.block<3,1>(0,3) + (1.0f-cpl_rel*cp_ap) *lp_H_rp_pgrasp.block<3,1>(0,3)) + (1.0f-reachable_p) * lr_H_rr_stb.block<3,1>(0,3); // TBC 
  Eigen::Vector3f d_p_rel = cpl_rel *(cp_ap * lp_H_rvp.block<3,1>(0,3) + (1.0f-cp_ap) *lp_H_rp_pgrasp.block<3,1>(0,3)) + (1.0f-cpl_rel) * lr_H_rr_stb.block<3,1>(0,3); // TBC 

  _error_rel.head(3) = lr_H_rr.block<3,1>(0,3) - d_p_rel;  // 

  // computing the velocity
  _V_rel.head(3) = -5.0f* gain_p_rel * _error_rel.head(3);  // 20
  _V_rel.tail(3) = -3.0f* jacMuTheta_rel.inverse() * gain_o_rel * _error_rel.tail(3);

  // ========================================
  // Computation of individual EE motion
  // ========================================
  // velocity
  float a_bi = 0.5f;
  float b_bi = 1.0f;
  Utils<float>::getBimanualTwistDistribution(a_bi, b_bi, _V_abs, _V_rel, Vd_ee[LEFT], Vd_ee[RIGHT]);

  Vd_ee[LEFT]  = Vd_ee[LEFT]  + _go2object * V_gpo[LEFT];
  Vd_ee[RIGHT] = Vd_ee[RIGHT] + _go2object * V_gpo[RIGHT];

  Vd_ee[LEFT]  = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[LEFT]);
  Vd_ee[RIGHT] = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[RIGHT]);

  std::cout << "[dualArmFreeMotionController]: --------------- cpl_rel ------- : \t"  << cpl_rel << std::endl;
  std::cout << "[dualArmFreeMotionController]: --------------- cp_ap ------- : \t"  << cp_ap << std::endl;

}

Eigen::Vector3f dualArmFreeMotionController::compute_modulated_motion(float activation, Eigen::Matrix3f BasisQ, Eigen::Vector3f Areach_ee, 
                                                                      Eigen::Vector3f Amodul_ee_norm, Eigen::Vector3f Amodul_ee_tang)
{
  //
  Eigen::MatrixXf den_temp  = Areach_ee.transpose() * Areach_ee;
  Eigen::RowVector3f Beta_j = 1.0f/(den_temp(0,0)+1e-10) * (Areach_ee.transpose() * BasisQ);

  Eigen::Matrix3f Lambda = Eigen::MatrixXf::Zero(3,3);
  Lambda.block<1,1>(0,0) = activation*( BasisQ.col(0).transpose() * Amodul_ee_tang * Beta_j(0) ) + (1.0-activation)*Eigen::MatrixXf::Identity(1,1);
  Lambda.block<1,1>(0,1) = activation*( BasisQ.col(0).transpose() * Amodul_ee_tang * Beta_j(1) );
  Lambda.block<1,1>(0,2) = activation*( BasisQ.col(0).transpose() * Amodul_ee_tang * Beta_j(2) );

  Lambda.block<1,1>(1,0) = activation*( BasisQ.col(1).transpose() * Amodul_ee_norm *Beta_j(0) );
  Lambda.block<1,1>(1,1) = activation*( BasisQ.col(1).transpose() * Amodul_ee_norm *Beta_j(1) )  + (1.0-activation)*Eigen::MatrixXf::Identity(1,1);
  Lambda.block<1,1>(1,2) = activation*( BasisQ.col(1).transpose() * Amodul_ee_norm *Beta_j(2) );

  Lambda.block<1,1>(2,0) = activation*( BasisQ.col(2).transpose() * Amodul_ee_norm *Beta_j(0) );
  Lambda.block<1,1>(2,1) = activation*( BasisQ.col(2).transpose() * Amodul_ee_norm *Beta_j(1) );
  Lambda.block<1,1>(2,2) = activation*( BasisQ.col(2).transpose() * Amodul_ee_norm *Beta_j(2) )  + (1.0-activation)*Eigen::MatrixXf::Identity(1,1);

  // computing the modulated second order DS (translation)
  return BasisQ * Lambda * BasisQ.transpose() * Areach_ee; 
}

Vector6f dualArmFreeMotionController::compute_modulated_motion_dual(float activation, Eigen::Matrix3f BasisQ[], Vector6f DS_ee_nominal, Vector6f Amodul_ee_norm, Vector6f Amodul_ee_tang)
{
  // computing the modulated second order DS (translation)
  Vector6f Vd_modulated = Eigen::VectorXf::Zero(6);
  Vd_modulated.head(3) = this->compute_modulated_motion(activation, BasisQ[LEFT],  DS_ee_nominal.head(3), Amodul_ee_norm.head(3), Amodul_ee_tang.head(3));
  Vd_modulated.tail(3) = this->compute_modulated_motion(activation, BasisQ[RIGHT], DS_ee_nominal.tail(3), Amodul_ee_norm.tail(3), Amodul_ee_tang.tail(3));

  return Vd_modulated; 
}


//
void dualArmFreeMotionController::dual_arm_motion(Eigen::Matrix4f w_H_ee[],  Vector6f Vee[], Eigen::Matrix4f w_H_gp[],  Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, Vector6f Vd_o,
                                                  Eigen::Matrix3f BasisQ[], Eigen::Vector3f VdImp[], bool isOrient3d, int taskType, Vector6f (&Vd_ee)[NB_ROBOTS], Eigen::Vector4f (&qd)[NB_ROBOTS], bool &release_flag)
{
  // States and desired states
  Eigen::Vector3f X[NB_ROBOTS],       // position of ee
                  Xdot[NB_ROBOTS],    // linear velocity of ee
                  Omega[NB_ROBOTS],   // angular velocity of ee
                  Xdes[NB_ROBOTS],    // desired position (final attractor)
                  Xb[NB_ROBOTS],      // first transitor attractor
                  Xe[NB_ROBOTS],      // final attractor
                  Xc[NB_ROBOTS];      // center of modulation ellipsoid
  //
  Eigen::Vector3f Xqb[NB_ROBOTS], Xqe[NB_ROBOTS];
  float dist2reach[NB_ROBOTS], 
        dist2line[NB_ROBOTS],
        dist2end[NB_ROBOTS];

  for(int i=0; i<NB_ROBOTS; i++){

    X[i]    = w_H_ee[i].block<3,1>(0,3);      
    Xdot[i] = Vee[i].head(3);     
    Omega[i]= Vee[i].tail(3);   
    Xdes[i] = w_H_gp[i].block<3,1>(0,3);      
    Xb[i]   = Xdes[i] + BasisQ[i]*Eigen::Vector3f(-0.5f*this->rho_, 0.0f, 0.0f);      
    Xe[i]   = Xdes[i] + BasisQ[i]*Eigen::Vector3f( 0.2f*this->rho_, 0.0f, 0.0f);      
    Xc[i]   = Xdes[i] + BasisQ[i]*Eigen::Vector3f(-0.2f*this->rho_, 0.0f, 0.0f);    

    //=======================================================================
    // Modulation term
    //=======================================================================
    Xqb[i] = BasisQ[i].transpose()*(X[i] - Xb[i]);
    Xqe[i] = BasisQ[i].transpose()*(X[i] - Xe[i]);

    dist2reach[i]   = (X[i] - Xc[i]).norm();
    dist2line[i]    = Xqb[i].tail(2).norm();
    dist2end[i]     = Xqe[i].head(1).norm();

  }

  // Modulation term
  a_proximity_  = 0.5f*(0.5f*(std::tanh(this->sw_proxim_* (0.5f*this->rho_ - dist2reach[LEFT]))  + 1.0f ) +       // scalar function of 3D distance  to 
                        0.5f*(std::tanh(this->sw_proxim_* (0.5f*this->rho_ - dist2reach[RIGHT])) + 1.0f ));       // initial (pre-modulation) position of attractor
  // a_proximity_  *= (1.0f-a_retract_);
  // a_proximity_ = 1.0;

  a_normal_   = a_proximity_* 0.5f*(0.5f*(std::tanh(this->sw_norm_  * (this->range_norm_ - dist2line[LEFT]))  + 1.0f ) +  // scalar function of distance to the line of direction VdImp 
                                    0.5f*(std::tanh(this->sw_norm_  * (this->range_norm_ - dist2line[RIGHT])) + 1.0f ));  // and passing through the release position

  a_tangent_  = a_proximity_* 0.5f*(0.5f*(std::tanh(this->sw_tang_  * (this->range_tang_ - dist2end[LEFT]))   + 1.0f ) +  // scalar function of distance to the 
                                    0.5f*(std::tanh(this->sw_tang_  * (this->range_tang_ - dist2end[RIGHT]))  + 1.0f ));  // stopping position of the throwing task
  
  // coupling_   = exp(-0.5f*(0.5f*dist2line[LEFT]+0.5f*dist2line[RIGHT])/(2.0f*range_norm_*range_norm_));
  // coupling_  = 1.0;
  if(a_tangent_ >= 0.95f){ 
    a_retract_   = 1.0f;
  }
  // 
  if((X[LEFT]-Xdes[LEFT]).norm() <= 1e-2 && (X[RIGHT]-Xdes[RIGHT]).norm() <= 1e-2){  // release if the norm is within 1 cm
    release_flag_ = true;
  }
  release_flag = release_flag_;  // reaching phase

  float activation   = a_proximity_;
  // activation = 0.0f;

  // state-dependent gain matrix
  // ----------------------------
    Eigen::Matrix4f w_H_gp_t[NB_ROBOTS];  // <----------------------
    w_H_gp_t[LEFT] = w_H_gp[LEFT];
    w_H_gp_t[LEFT].block<3,1>(0,3) = Xb[LEFT];
    //
    w_H_gp_t[RIGHT] = w_H_gp[RIGHT];
    w_H_gp_t[RIGHT].block<3,1>(0,3) = Xb[RIGHT];

    std::cout << " POSITION : ---------- [w_H_gp_t] \t" << w_H_gp_t[LEFT].block<3,1>(0,3).transpose() << " and \t" << w_H_gp_t[RIGHT].block<3,1>(0,3).transpose() << std::endl;
    Vector6f Vd_ee_nom[NB_ROBOTS];
    Eigen::Vector4f qd_nom[NB_ROBOTS];
    //
    if(_modulated_reaching || _isNorm_impact_vel){
      this->computeCoordinatedMotion2(w_H_ee,  w_H_gp_t, w_H_o, Vd_ee_nom, qd_nom, isOrient3d);
    }
    else{
      this->computeCoordinatedMotion2(w_H_ee,  w_H_gp, w_H_o, Vd_ee_nom, qd_nom, isOrient3d);
    }
    
    // this->computeCoordinatedMotion2(w_H_ee,  w_H_gp_t, w_H_o, Vd_ee_nom, qd_nom, isOrient3d);
    //
    Vector6f DS_ee_nominal = Eigen::VectorXf::Zero(6);
    DS_ee_nominal.head(3)  = Vd_ee_nom[LEFT].head(3);
    DS_ee_nominal.tail(3)  = Vd_ee_nom[RIGHT].head(3);
    
    // //
    Matrix6f A = Eigen::MatrixXf::Identity(6,6);
    A.block<3,3>(0,0) = -4.0f * this->gain_p_abs;
    A.block<3,3>(3,3) = -4.0f * this->gain_p_rel;
    Matrix6f A_prime  = _Tbi.inverse() * A * _Tbi;
    Matrix6f _Tbi_1_A = _Tbi.inverse() * A;
    //
    Vector6f X_dual, Xdes_dual, Xb_dual, VdImp_dual;

    X_dual.head(3)      = X[LEFT];
    X_dual.tail(3)      = X[RIGHT];
    Xdes_dual.head(3)   = Xdes[LEFT];
    Xdes_dual.tail(3)   = Xdes[RIGHT];
    Xb_dual.head(3)     = Xb[LEFT];
    Xb_dual.tail(3)     = Xb[RIGHT];
    VdImp_dual.head(3)  = VdImp[LEFT];
    VdImp_dual.tail(3)  = VdImp[RIGHT];
    //
    Eigen::Matrix3f Q_toss =  Eigen::MatrixXf::Identity(3,3);
    if(Vd_o.head(3).norm() <= 1e-6){
      Q_toss  = Utils<float>::create3dOrthonormalMatrixFromVector(Eigen::Vector3f(1.0f, 0.0f, 0.0f)); //
    }
    else{
      Q_toss  = Utils<float>::create3dOrthonormalMatrixFromVector(Vd_o.head(3)); //
    }
    Eigen::Vector3f Xqo  = Q_toss.transpose()*(w_H_o.block<3,1>(0,3) - w_H_Do.block<3,1>(0,3));
    float dist2line_toss = Xqo.tail(2).norm();
    a_normal_Do_ = 0.5f*(std::tanh(1.0f*this->sw_norm_  * (0.99f*this->range_norm_ - dist2line_toss))  + 1.0f );
    //
    if(a_normal_Do_ >=0.90f){
      a_release_ = 1.0f;
    }
    //
    float sw_norm_Do = (a_normal_Do_ + a_release_);
    if((a_normal_Do_ + a_release_) >= 1.0f){
      sw_norm_Do = 1.0f;
    }
    sw_norm_Do = 1.0f;

    //
    Vector6f Xstar_dual = X_dual;
    Vector6f Amodul_ee_norm = Eigen::VectorXf::Zero(6); //A_prime*(X_dual - Xb_dual);          // Modulated DS that aligned  the EE with the desired velocity
    Vector6f Amodul_ee_tang = Eigen::VectorXf::Zero(6); //A_prime*(X_dual - Xstar_dual);  // ;
    //
    switch(taskType){
      case 0: {   // reaching with impact
        Xstar_dual = (1.0 - a_normal_) * Xb_dual + a_normal_ * (X_dual - A_prime.inverse()*VdImp_dual);  // TO DO: add 
        if(VdImp[LEFT].norm() <= 0.01f || VdImp[RIGHT].norm() <= 0.01f){
          Xstar_dual = (1.0 - a_normal_) * Xb_dual + a_normal_ * Xdes_dual;  // TO DO: add 
        }
        //
        Amodul_ee_norm = A_prime*(X_dual - Xb_dual);          // Modulated DS that aligned  the EE with the desired velocity
        Amodul_ee_tang = A_prime*(X_dual - Xstar_dual);  // ;

        _refVtoss_EE   = 0.0;
        if(!(_modulated_reaching || _isNorm_impact_vel)){
          activation = 0.0f;
        }
        // _integral_Vee_d[LEFT].setZero();
        // _integral_Vee_d[RIGHT].setZero();
        // activation = 0.0f;
      }
      break;
      case 1:{ // point to point motion of the object 
        Vector6f X_bi = Eigen::VectorXf::Zero(6);
        X_bi.head(3)  = w_H_Do.block<3,1>(0,3) - w_H_o.block<3,1>(0,3)+ 0.5f*(X[LEFT] + X[RIGHT]);
        X_bi.tail(3)  = 0.99f*(X[RIGHT] - X[LEFT]);

        Xstar_dual =  _Tbi.inverse() * X_bi;
        // Amodul_ee_norm = _Tbi.inverse() * A * _Tbi *(X_dual - Xstar_dual);          // Modulated DS that aligned  the EE with the desired velocity
        // Amodul_ee_tang = _Tbi.inverse() * A * _Tbi *(X_dual - Xstar_dual);  // ;

        Vector6f v_task_bi = ( A * _Tbi*(X_dual - Xstar_dual));
                 v_task_bi = Utils<float>::SaturationTwist(Vd_o.head(3).norm(), _w_max, v_task_bi);
        // v_task_bi.head(3)  = v_task_bi.head(3)/(v_task_bi.head(3).norm()+1e-10)*1.0f*v_task_bi.head(3).norm();
        // v_task_bi.head(3) = v_task_bi.head(3)/(v_task_bi.head(3).norm()+1e-10)*Vd_o.head(3).norm();

        Amodul_ee_norm = _Tbi.inverse() * v_task_bi;  // 
        Amodul_ee_tang = _Tbi.inverse() * v_task_bi;  //
        //
        activation = 1.0f;

        //
        // this->constrained_ang_vel_correction(w_H_ee, w_H_gp, w_H_o, w_H_Do, Vd_ee_nom, true);
      }
      break;

      case 2:{ //  velocity based motion of the object
        //
        _integral_Vee_d[LEFT].setZero();
        _integral_Vee_d[RIGHT].setZero();
        //
        Vector6f X_bi = Eigen::VectorXf::Zero(6);
        X_bi.head(3)  = 0.5f*(X[LEFT] + X[RIGHT]);
        X_bi.tail(3)  = 0.95f*(X[RIGHT] - X[LEFT]);
        Xstar_dual =  _Tbi.inverse() * X_bi;
        //velocity based motion of the object
        Vector6f Xdot_bi = Eigen::VectorXf::Zero(6);
        Eigen::Vector3f X_rel = X[RIGHT] - X[LEFT];
        Eigen::Vector3f w_o   = 0.0f*Vd_o.tail(3);

        Xdot_bi.head(3)  = Vd_o.head(3);
        Xdot_bi.tail(3)  = w_o.cross(X_rel);

        Vector6f v_task_bi = ( A * _Tbi*(X_dual - Xstar_dual) + 1.0f*sw_norm_Do* Xdot_bi );
        v_task_bi.head(3) = v_task_bi.head(3)/(v_task_bi.head(3).norm()+1e-10)*Vd_o.head(3).norm();

        Amodul_ee_norm = _Tbi.inverse() * v_task_bi;  // 
        Amodul_ee_tang = _Tbi.inverse() * v_task_bi;  //
        //
        activation = 1.0f;
      }
      break;

      case 3:{ // point to point motion of the object 
        //
        _integral_Vee_d[LEFT].setZero();
        _integral_Vee_d[RIGHT].setZero();
        //
        Vector6f X_bi = Eigen::VectorXf::Zero(6);
        X_bi.head(3)  = 0.5f*(X[LEFT] + X[RIGHT]);
        X_bi.tail(3)  = 0.95f*(X[RIGHT] - X[LEFT]);
        Xstar_dual    =  _Tbi.inverse() * X_bi;
        //
        //velocity based motion of the object
        Vector6f Xdot_bi      = Eigen::VectorXf::Zero(6);
        Eigen::Vector3f X_rel = X[RIGHT] - X[LEFT];

        Eigen::Vector3f w_o   = 0.0f*Vd_o.tail(3);
        Xdot_bi.head(3)       = Vd_o.head(3);
        Xdot_bi.tail(3)       = w_o.cross(X_rel);

        Vector6f v_task_bi = ( A * _Tbi*(X_dual - Xstar_dual) + 1.0f*sw_norm_Do* Xdot_bi );
        v_task_bi.head(3)  = v_task_bi.head(3)/(v_task_bi.head(3).norm()+1e-10)*1.0f*v_task_bi.head(3).norm();
        // v_task_bi.head(3) = v_task_bi.head(3)/(v_task_bi.head(3).norm()+1e-10)*Vd_o.head(3).norm();

        Amodul_ee_norm = _Tbi.inverse() * v_task_bi;  // 
        Amodul_ee_tang = _Tbi.inverse() * v_task_bi;  //
        //
        activation = 1.0f;
      }
      break;

      case 4:{ // point to point motion of the object 
        //
        _integral_Vee_d[LEFT].setZero();
        _integral_Vee_d[RIGHT].setZero();
        //
        Vector6f X_bi = Eigen::VectorXf::Zero(6);
        X_bi.head(3)  = 0.50f*(X[LEFT] + X[RIGHT]);
        X_bi.tail(3)  = 0.95f*(X[RIGHT] - X[LEFT]);
        Xstar_dual    =  _Tbi.inverse() * X_bi;
        //
        //velocity based motion of the object
        Vector6f Xdot_bi      = Eigen::VectorXf::Zero(6);
        Eigen::Vector3f X_rel = X[RIGHT] - X[LEFT];
        Vector6f Vo = Eigen::VectorXf::Zero(6);
        Vector6f Vo_place = this->generatePlacingMotion2(w_H_o, w_H_Do, _height_via_point, Vo);
        float cp_obj = Utils<float>::computeCouplingFactor(w_H_o.block<3,1>(0,3)-w_H_Do.block<3,1>(0,3), 50.0f, 0.12f, 1.0f, true);
        // float cp_obj = 0.5f*(std::tanh(1.5f*this->sw_norm_  * (1.0f*this->range_norm_ - (w_H_o.block<3,1>(0,3)-w_H_Do.block<3,1>(0,3)).norm()))  + 1.0f );
        if(false){ // false
          Vo_place.head(3) = Vo_place.head(3).normalized() * (cp_obj*Vo_place.head(3).norm() +(1.0f -cp_obj)*Vd_o.head(3).norm());
        }
        else{
          Vo_place.head(3) = Vo_place.head(3).normalized() * (Vd_o.head(3).norm());
        }
        // 

        Eigen::Vector3f w_o   = 0.0f*Vo_place.tail(3);
        Xdot_bi.head(3)       = Vo_place.head(3);
        Xdot_bi.tail(3)       = w_o.cross(X_rel);
       
        Vector6f v_task_bi = ( A * _Tbi*(X_dual - Xstar_dual) + Xdot_bi );

        // v_task_bi.head(3) = v_task_bi.head(3)/(v_task_bi.head(3).norm()+1e-10)*(cp_obj*v_task_bi.head(3).norm() +(1.0f -cp_obj)*Vd_o.head(3).norm());
        // v_task_bi.head(3) = v_task_bi.head(3)/(v_task_bi.head(3).norm()+1e-10)*(v_task_bi.head(3).norm());
        v_task_bi.head(3) = v_task_bi.head(3)/(v_task_bi.head(3).norm()+1e-10)*Vd_o.head(3).norm();

        Amodul_ee_norm = _Tbi.inverse() * v_task_bi;  // 
        Amodul_ee_tang = _Tbi.inverse() * v_task_bi;  //
        //
        activation = 1.0f;
        //
        this->computeDesiredOrientation(1.0f, w_H_ee, w_H_gp, w_H_o, qd_nom, false);

        //
        Vd_ee_nom[LEFT].tail(3)  = Vo_place.tail(3);
        Vd_ee_nom[RIGHT].tail(3) = Vo_place.tail(3);
      }
      break;
    }   
    // get the modulated motion (out_motion: Velocity)
    Vector6f DS_ee_modulated = Eigen::VectorXf::Zero(6,1);
    DS_ee_modulated = this->compute_modulated_motion_dual(activation, BasisQ, DS_ee_nominal, Amodul_ee_norm, Amodul_ee_tang);

    Vd_ee[LEFT].head(3) = DS_ee_modulated.head(3);
    Vd_ee[LEFT].tail(3) = Vd_ee_nom[LEFT].tail(3);
    //
    Vd_ee[RIGHT].head(3) = DS_ee_modulated.tail(3);
    Vd_ee[RIGHT].tail(3) = Vd_ee_nom[RIGHT].tail(3);
    //

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Unitary velocity field
    float speed_ee[2];
 
    Eigen::Vector3f o_error_pos_abs_paral = this->getAbsoluteTangentError(w_H_o, w_H_ee, w_H_gp);
    float cp_ap = Utils<float>::computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.05f, 1.5f, true);  // 50.0f, 0.05f, 2.8f /  50.0f, 0.15f, 1.0f
    float cp_ap2 = 0.0f;
    float  alp  = 1.0f; //0.05f;
    
    if(_modulated_reaching){
      if(true){
        alp = 0.10f;
      }
      cp_ap2     = 0.0f;
      // cp_ap = 0.0f;
      // _refVreach[LEFT]  = (1.0f-alp)*_refVreach[LEFT]  + alp*((1.0f-cp_ap)*_desVreach + cp_ap* VdImp[LEFT].norm());
      // _refVreach[RIGHT] = (1.0f-alp)*_refVreach[RIGHT] + alp*((1.0f-cp_ap)*_desVreach + cp_ap* VdImp[RIGHT].norm());
      _refVreach[LEFT]  = (1.0f-alp)*_refVreach[LEFT]  + alp*((1.0f-cp_ap)*DS_ee_modulated.head(3).norm() + cp_ap*VdImp[LEFT].norm());
      _refVreach[RIGHT] = (1.0f-alp)*_refVreach[RIGHT] + alp*((1.0f-cp_ap)*DS_ee_modulated.tail(3).norm() + cp_ap*VdImp[RIGHT].norm());
      speed_ee[LEFT]  = _refVreach[LEFT];
      speed_ee[RIGHT] = _refVreach[RIGHT];

      std::cout << " XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX HERE IN MOD REACH XXXXXXXXXXXXXXXX " << std::endl;
    }
    else if(_isNorm_impact_vel){
      cp_ap2     = 0.0f;
      _refVreach[LEFT]  = (1.0f-cp_ap)*DS_ee_modulated.head(3).norm() + cp_ap*VdImp[LEFT].norm();
      _refVreach[RIGHT] = (1.0f-cp_ap)*DS_ee_modulated.tail(3).norm() + cp_ap*VdImp[RIGHT].norm();
      speed_ee[LEFT]    = _refVreach[LEFT];
      speed_ee[RIGHT]   = _refVreach[RIGHT];
      std::cout << " IIIIIIIIIIIIIIIIIIIIIIIIIIIIII HERE IN NORM IMPACT IIIIIIIIIIIIIII " << std::endl;
    }
    else{
      cp_ap2 = 0.0f*Utils<float>::computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.02f, 1.2f, true);
       alp = 0.10f;
      _refVreach[LEFT]  = (1.0f-alp)*_refVreach[LEFT]  + alp*(DS_ee_nominal.head(3).norm());
      _refVreach[RIGHT] = (1.0f-alp)*_refVreach[RIGHT] + alp*(DS_ee_nominal.tail(3).norm());
      speed_ee[LEFT]    = _refVreach[LEFT];
      speed_ee[RIGHT]   = _refVreach[RIGHT];

      std::cout << " NNNNNNNNNNNNNNNNNNNNNNNNNNNNNN HERE IN CLASSICAL NNNNNNNNNNNNNNNNN " << std::endl;
    }
    //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    if(taskType == 0){
      speed_ee[LEFT]  = _refVreach[LEFT];
      speed_ee[RIGHT] = _refVreach[RIGHT];
      // speed_ee[LEFT]  = DS_ee_modulated.head(3).norm();
      // speed_ee[RIGHT] = DS_ee_modulated.tail(3).norm();
    }
    else{
      speed_ee[LEFT]  = DS_ee_modulated.head(3).norm();
      speed_ee[RIGHT] = DS_ee_modulated.tail(3).norm();
    }
    //
    Vd_ee[LEFT].head(3)  = Vd_ee[LEFT].head(3)/(Vd_ee[LEFT].head(3).norm()+1e-10)  * speed_ee[LEFT]; 
    Vd_ee[RIGHT].head(3) = Vd_ee[RIGHT].head(3)/(Vd_ee[RIGHT].head(3).norm()+1e-10)* speed_ee[RIGHT]; 
    //
    // Vd_ee[LEFT]  = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[LEFT]);
    // Vd_ee[RIGHT] = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[RIGHT]);
    qd[LEFT]  = qd_nom[LEFT];
    qd[RIGHT] = qd_nom[RIGHT];
}


Eigen::Vector3f dualArmFreeMotionController::getAbsoluteTangentError(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_gp[]){

  Eigen::Vector3f normal_l = w_H_gp[0].block<3,1>(0,2);
  Eigen::Matrix3f oSpace   = Utils<float>::orthogonalProjector(normal_l);
  //
  Eigen::Vector3f error_p_abs     = w_H_o.block(0,3,3,1) - 0.5f*( w_H_ee[LEFT].block(0,3,3,1) +  w_H_ee[RIGHT].block(0,3,3,1));
  
  return  oSpace * error_p_abs;
}

void dualArmFreeMotionController::compute_EE_avoidance_velocity(Eigen::Matrix4f w_H_ee[], Vector6f (&VEE_oa)[NB_ROBOTS]){

  Eigen::Vector3f center_ssphere_EE_l = w_H_ee[LEFT].block(0,3,3,1) +(0.5f*_min_dist_EE - _safe_radius)*w_H_ee[LEFT].block(0,0,3,3).col(2);
  Eigen::Vector3f center_ssphere_EE_r = w_H_ee[RIGHT].block(0,3,3,1) +(0.5f*_min_dist_EE - _safe_radius)*w_H_ee[RIGHT].block(0,0,3,3).col(2);

  float dist_EE = (center_ssphere_EE_l -  center_ssphere_EE_r).norm() - 2.0f * _safe_radius;
  float alpha_active = 0.5f*(std::tanh(_sw_EE_obsAv  * (_min_dist_EE - dist_EE)) + 1.0 );
  //
  Eigen::Vector3f uEE_LR = w_H_ee[RIGHT].block(0,3,3,1) -  w_H_ee[LEFT].block(0,3,3,1);
  Eigen::Vector3f uEE_RL = w_H_ee[LEFT].block(0,3,3,1) -  w_H_ee[RIGHT].block(0,3,3,1);
  VEE_oa[LEFT].head(3)  = -alpha_active * _v_max * (w_H_ee[RIGHT].block(0,3,3,1) -  w_H_ee[LEFT].block(0,3,3,1)).normalized();
  VEE_oa[RIGHT].head(3) = -alpha_active * _v_max * (w_H_ee[LEFT].block(0,3,3,1) -  w_H_ee[RIGHT].block(0,3,3,1)).normalized();
}

void dualArmFreeMotionController::constrained_ang_vel_correction(Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_gp[], Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_Do, Vector6f (&VEE)[NB_ROBOTS], bool wIntegral){
  // compute angular velocity for the object (from current to desired)
  // object orientation error
  Eigen::Matrix3f do_R_o   = w_H_Do.block<3,3>(0,0);
  Eigen::Matrix4f w_H_o_t  = w_H_o;
  w_H_o_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(1.0f, w_H_o.block<3,3>(0,0), do_R_o); //desired
  // relative transformation between desired and current frame
  Eigen::Matrix4f do_H_o = w_H_o_t.inverse() * w_H_o;
  //
  Eigen::Vector3f error_o       = Utils<float>::getPoseErrorCur2Des(do_H_o).tail(3);
  // 3D Orientation Jacobian 
  Eigen::Matrix3f jacMuTheta_o = Utils<float>::getMuThetaJacobian(do_H_o.block<3,3>(0,0)) * w_H_o.block<3,3>(0,0).transpose();
  //
  // ---------------------------------
  // computing of desired ee velocity
  // ---------------------------------
  Omega_object_d_ = -1.2f* jacMuTheta_o.inverse() * gain_o_abs * error_o;
  //
  for(int k=0; k<NB_ROBOTS; k++){
    //
    Eigen::Vector3f tog = w_H_o.block<3,1>(0,3) - w_H_gp[k].block<3,1>(0,3);
    Eigen::Matrix3f skew_Mx_og; 
    skew_Mx_og <<   0.0f,   -tog(2),   tog(1),
                     tog(2),   0.0f,  -tog(0),
                    -tog(1), tog(0),     0.0f;

    VEE[k].head(3) =  VEE[k].head(3) + 1.0f*skew_Mx_og * Omega_object_d_;  
    VEE[k].tail(3) =  VEE[k].tail(3) + 1.0f * Omega_object_d_;  

    if(wIntegral){
      _integral_Vee_d[k] = _integral_Vee_d[k] + 0.5f*_dt*VEE[k].tail(3); 
      if(_integral_Vee_d[k].norm() > 0.5f){
        _integral_Vee_d[k] = _integral_Vee_d[k].normalized() * 0.5f;
      }
    }
    else{
      _integral_Vee_d[k].setZero();
    }
    VEE[k].tail(3) =  2.0*VEE[k].tail(3) + _integral_Vee_d[k];

    // std::cout << " WWWWWWWWWWWWWWWWWWWWWWWWW Wo " << k << " is \t " << Omega_object_d_.transpose() << std::endl; 

    VEE[k] = Utils<float>::SaturationTwist(_v_max, _w_max, VEE[k]);      
  }
  
}


void dualArmFreeMotionController::updateDesiredGraspingPoints(bool no_dual_mds_method, 
                                                              bool isPlacing,
                                                              bool isThrowing,
                                                              bool isClose2Release,
                                                              Eigen::Vector3f xgp_o[],
                                                              Eigen::Vector4f qgp_o[],
                                                              Eigen::Matrix4f o_H_ee[],
                                                              Eigen::Matrix4f w_H_o,
                                                              Eigen::Matrix4f &w_H_Do,
                                                              Eigen::Vector3f xDo_placing,
                                                              Eigen::Vector4f qDo_placing,
                                                              Eigen::Vector3f release_position,
                                                              Eigen::Vector4f release_orientation,
                                                              Eigen::Matrix4f &w_H_DesObj,
                                                              Eigen::Matrix4f (&w_H_gp)[NB_ROBOTS],
                                                              Eigen::Matrix4f (&w_H_Dgp)[NB_ROBOTS])
{
  //
  // Eigen::Matrix4f w_H_gp[NB_ROBOTS];
  for(int k=0; k<NB_ROBOTS; k++){
    w_H_gp[k]  = w_H_o * Utils<float>::pose2HomoMx(xgp_o[k],  qgp_o[k]);
  }
  //
  w_H_DesObj  = w_H_Do; //
  //
  if(isPlacing){
    w_H_DesObj  = Utils<float>::pose2HomoMx(xDo_placing, qDo_placing); // w_H_Do = w_H_DesObj
  }

  if(isThrowing){
    w_H_DesObj  = Utils<float>::pose2HomoMx(release_position, release_orientation);
  }

  w_H_Dgp[LEFT].block(0,0,3,3)  = w_H_DesObj.block(0,0,3,3) * Utils<float>::pose2HomoMx(xgp_o[LEFT],  qgp_o[LEFT]).block(0,0,3,3);
  w_H_Dgp[RIGHT].block(0,0,3,3) = w_H_DesObj.block(0,0,3,3) * Utils<float>::pose2HomoMx(xgp_o[RIGHT],  qgp_o[RIGHT]).block(0,0,3,3);
  //
  if(isThrowing && isClose2Release){
    w_H_Dgp[LEFT]  = w_H_DesObj * o_H_ee[LEFT];
    w_H_Dgp[RIGHT] = w_H_DesObj * o_H_ee[RIGHT];
    // w_H_Dgp[LEFT]  = w_H_o * o_H_ee[LEFT];
    // w_H_Dgp[RIGHT] = w_H_o * o_H_ee[RIGHT];
  }

  if(!no_dual_mds_method){
    w_H_Dgp[LEFT].block(0,0,3,3)  = w_H_o.block(0,0,3,3) * Utils<float>::pose2HomoMx(xgp_o[LEFT],  qgp_o[LEFT]).block(0,0,3,3);
    w_H_Dgp[RIGHT].block(0,0,3,3) = w_H_o.block(0,0,3,3) * Utils<float>::pose2HomoMx(xgp_o[RIGHT],  qgp_o[RIGHT]).block(0,0,3,3);
    if(isClose2Release){
      w_H_Dgp[LEFT]  = w_H_o * o_H_ee[LEFT];
      w_H_Dgp[RIGHT] = w_H_o * o_H_ee[RIGHT];
    }
  }

}


void dualArmFreeMotionController::getDesiredMotion( bool no_dual_mds_method,
                                                    bool isContact, 
                                                    bool isPlacing,
                                                    bool isThrowing,
                                                    bool isClose2Release,
                                                    int dualTaskSelector,
                                                    Eigen::Matrix4f w_H_ee[], 
                                                    Eigen::Vector3f xgp_o[],
                                                    Eigen::Vector4f qgp_o[],
                                                    Eigen::Matrix4f o_H_ee[],
                                                    Eigen::Matrix4f w_H_o, 
                                                    Eigen::Matrix4f &w_H_Do,
                                                    Eigen::Vector3f xDo_placing,
                                                    Eigen::Vector4f qDo_placing,
                                                    Eigen::Vector3f release_position,
                                                    Eigen::Vector4f release_orientation,
                                                    float height_via_point,
                                                    Vector6f Vee[],
                                                    Vector6f Vd_o,
                                                    Eigen::Matrix3f BasisQ[],
                                                    Eigen::Vector3f VdImpact[],
                                                    Eigen::Matrix4f (&w_H_Dgp)[NB_ROBOTS],
                                                    Vector6f (&Vd_ee)[NB_ROBOTS], 
                                                    Eigen::Vector4f (&qd)[NB_ROBOTS], 
                                                    bool &release_flag)
{
  //
  Eigen::Matrix4f w_H_DesObj;
  Eigen::Matrix4f w_H_gp[NB_ROBOTS];
  // Eigen::Matrix4f w_H_Dgp[NB_ROBOTS];
  //
  Eigen::Matrix4f w_H_Dobject = w_H_Do;
  //
  this->updateDesiredGraspingPoints(no_dual_mds_method, 
                                    isPlacing,
                                    isThrowing,
                                    isClose2Release,
                                    xgp_o,
                                    qgp_o,
                                    o_H_ee,
                                    w_H_o,
                                    w_H_Dobject,
                                    xDo_placing,
                                    qDo_placing,
                                    release_position,
                                    release_orientation,
                                    w_H_DesObj,
                                    w_H_gp,
                                    w_H_Dgp);

  if(isContact)  // constrained motion : lifting, placing and tossing
  {
    if(no_dual_mds_method)
    {
      this->computeConstrainedMotion(w_H_ee, w_H_Dgp, w_H_o, Vd_ee, qd, false);

      if(isPlacing){
          this->generatePlacingMotion(w_H_ee, w_H_Dgp,  w_H_o, w_H_DesObj, height_via_point, Vd_ee, qd, false);
      }
      if(isThrowing){
        this->computeConstrainedMotion(w_H_ee, w_H_Dgp, w_H_o, Vd_ee, qd, false);
      }
    }
    else  // mds
    {
      // dualTaskSelector : 0=reach, 1=pick, 2=toss, 3=pick_and_toss, 4=pick_and_place
      if(isPlacing || isThrowing){
        w_H_Dobject = w_H_DesObj;
      } 
      this->dual_arm_motion(w_H_ee,  Vee, w_H_Dgp,  w_H_o, w_H_Dobject, Vd_o, BasisQ, VdImpact, false, dualTaskSelector, Vd_ee, qd, release_flag); 
    }
  }
  else // Free-motion: reaching
  {
    if(false || no_dual_mds_method)
    {
      this->computeCoordinatedMotion2(w_H_ee, w_H_gp, w_H_o, Vd_ee, qd, false);
      //
      Eigen::Vector3f error_p_abs     = w_H_o.block(0,3,3,1) - 0.5f*( w_H_ee[LEFT].block(0,3,3,1) +  w_H_ee[RIGHT].block(0,3,3,1));
      Eigen::Vector3f o_error_pos_abs = w_H_o.block<3,3>(0,0).transpose() * error_p_abs;
      Eigen::Vector3f o_error_pos_abs_paral = Eigen::Vector3f(o_error_pos_abs(0), 0.0f, o_error_pos_abs(2));
      float cp_ap = Utils<float>::computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.17f, 1.0f, true);  // 50.0f, 0.05f, 2.8f
      // create impact in the normal direction
      Vd_ee[LEFT].head(3)  = Vd_ee[LEFT].head(3)  + w_H_gp[LEFT].block(0,0,3,3).col(2)  * cp_ap * 0.20f; //     n[k] = _w_H_gp[k].block(0,0,3,3).col(2);
      Vd_ee[RIGHT].head(3) = Vd_ee[RIGHT].head(3) + w_H_gp[RIGHT].block(0,0,3,3).col(2) * cp_ap * 0.20f; //
    }
    else  // mds
    {
      this->dual_arm_motion(w_H_ee,  Vee, w_H_gp,  w_H_o, w_H_Dobject, Vd_o, BasisQ, VdImpact, false, 0, Vd_ee, qd, release_flag);    // 0: reach
    }
  }
}


Eigen::Vector2f dualArmFreeMotionController::estimateRobot_PathLength_AverageSpeed(throwingDS &dsThrowing,
                                                                                    bool no_dual_mds_method, 
                                                                                    bool isPlacing, 
                                                                                    bool isThrowing, 
                                                                                    int dualTaskSelector, 
                                                                                    float dt,
                                                                                    float desVimp,
                                                                                    float tolerance_dist2contact,
                                                                                    float height_via_point,
                                                                                    Eigen::Vector3f xDo_placing,
                                                                                    Eigen::Vector4f qDo_placing,
                                                                                    Eigen::Vector3f release_position,  
                                                                                    Eigen::Vector4f release_orientation,
                                                                                    Eigen::Vector3f xgp_o[],
                                                                                    Eigen::Vector4f qgp_o[],
                                                                                    Eigen::Vector3f VdImpact[],
                                                                                    Eigen::Matrix3f BasisQ[], 
                                                                                    Eigen::Matrix4f w_H_Do,
                                                                                    Eigen::Matrix4f w_H_o,
                                                                                    Eigen::Matrix4f w_H_Dgp[],
                                                                                    Eigen::Matrix4f w_H_ee[])
{
  
  bool isReleasePositionReached = false;
  bool release_flag = false;
  bool isContact    = false;
  bool isClose2Release  = false;
  bool isPlacingCommand = false;
  bool isTossingCommand = false;
  float tol_dist2contact = tolerance_dist2contact;
  float isPickupSetEstim = false;
  //
  int max_horizon = 15;
  int pred_count  = 0;

  Vector6f Vd_ee[NB_ROBOTS];
  Eigen::Vector4f qd[NB_ROBOTS];

  dsThrowing._refVtoss = desVimp;
  dsThrowing.reset_release_flag();
  Eigen::Vector3f xo = w_H_o.block(0,3,3,1);
  Eigen::Vector4f qo = Utils<float>::rotationMatrixToQuaternion(w_H_o.block(0,0,3,3));
  Vector6f Vee[NB_ROBOTS];
            Vee[LEFT].setZero();
            Vee[RIGHT].setZero();
  Eigen::Vector3f vo = Eigen::VectorXf::Zero(3);
  Vector6f Vd_o = Eigen::VectorXf::Zero(6);
  //
  Eigen::Matrix4f w_H_obj = w_H_o;

  Eigen::Matrix4f w_H_EE[NB_ROBOTS], o_H_ee[NB_ROBOTS], w_H_Dgpts[NB_ROBOTS], w_H_gpts[NB_ROBOTS];
  for(int k=0; k<NB_ROBOTS; k++){
    Vd_ee[k].setZero();
    w_H_EE[k]   = w_H_ee[k];
    o_H_ee[k]   = w_H_obj.inverse() * w_H_ee[k];
    w_H_gpts[k] = w_H_obj * Utils<float>::pose2HomoMx(xgp_o[k],  qgp_o[k]);
    w_H_Dgpts[k]= w_H_Dgp[k];
  }

  //
  std::vector<Eigen::Vector3f> X_next;
  std::vector<Eigen::Vector3f> dX_next;
  Eigen::Vector3f xEE,  xEE_0, dxEE; 
  xEE_0  = 0.5f*(w_H_EE[LEFT].block(0,3,3,1)+w_H_EE[RIGHT].block(0,3,3,1));
  
  //
  while((!isReleasePositionReached) && (pred_count<max_horizon))
  {
    
    // -------------------------------------------------------------------------------------
    if(!isPickupSetEstim){
      if(isContact){
        dsThrowing.set_pickup_object_pose(xo, qo);
        Eigen::Vector3f new_toss_velocity = dsThrowing.v_toss_.norm() * (release_position -xo).normalized();
        dsThrowing.set_toss_linear_velocity(new_toss_velocity);
        isPickupSetEstim = true;
      }
      else{
        dsThrowing.set_pickup_object_pose(xo, qo);
      }

    }

    if(isContact){
      Vd_o  = dsThrowing.apply(xo, qo, vo, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1);  // Function to call in a loop
      isReleasePositionReached = dsThrowing.get_release_flag();
    }
    else
    {
      dsThrowing._refVtoss = desVimp;
      Vd_o.setZero();  // for data logging
    }
    // -------------------------------------------------------------------------------------
    this->getDesiredMotion(no_dual_mds_method,          // preset
                            isContact,                    // to be updated
                            isPlacing,                    // preset
                            isThrowing,                   // preset
                            isClose2Release,              // to be updated
                            dualTaskSelector,             // preset
                            w_H_EE,                       // initialized and updated
                            xgp_o,                        // fixed
                            qgp_o,                        // fixed
                            o_H_ee,                       // updated
                            w_H_obj,                      // initialized and updated
                            w_H_Do,                       // preset
                            xDo_placing,                  // fixed
                            qDo_placing,                  // fixed
                            release_position,             // preset
                            release_orientation,          // preset
                            height_via_point,             // preset
                            Vee,                          // updated
                            Vd_o,                         // updated
                            BasisQ,                       // preset
                            VdImpact,                     // preset
                            w_H_Dgpts,                    // initialized and updated
                            Vd_ee,                        // output updated
                            qd,                           // output updated
                            release_flag);                // output
    // --------------------------------------------------------------------------------------
    // update pose from velocity
    // --------------------------
    Utils<float>::UpdatePose_From_VelocityTwist(dt, Vd_ee[LEFT], w_H_EE[LEFT]);
    Utils<float>::UpdatePose_From_VelocityTwist(dt, Vd_ee[RIGHT], w_H_EE[RIGHT]);
    Utils<float>::UpdatePose_From_VelocityTwist(dt, Vd_o, w_H_obj);

    
    for(int k=0; k<NB_ROBOTS; k++){
      o_H_ee[k]     = w_H_obj.inverse() * w_H_EE[k];
      w_H_gpts[k]   = w_H_obj * Utils<float>::pose2HomoMx(xgp_o[k],  qgp_o[k]);
      w_H_Dgpts[k]  = w_H_Do * o_H_ee[k];
    }

    xo = w_H_obj.block(0,3,3,1);
    qo = Utils<float>::rotationMatrixToQuaternion(w_H_obj.block(0,0,3,3));
    vo = Vd_o.head(3);

    // update contact status
      // Positioning error in hand frames
      Eigen::Vector3f lh_er = w_H_gpts[LEFT].block<3,3>(0,0).transpose()*(w_H_gpts[LEFT].block<3,1>(0,3)-w_H_EE[LEFT].block<3,1>(0,3));   // 
      Eigen::Vector3f rh_er = w_H_gpts[RIGHT].block<3,3>(0,0).transpose()*(w_H_gpts[RIGHT].block<3,1>(0,3)-w_H_EE[RIGHT].block<3,1>(0,3));  // 
    
      // Distances normal to contacts: lh_er(2), rh_er(2)
      if((fabs(lh_er(2)) <= tol_dist2contact) && (fabs(rh_er(2)) <= tol_dist2contact)){ 
        isContact = true;
      }
      else{
        isContact = false;
      }

    // update Convergence status
    isClose2Release  = (dsThrowing.a_tangent_> 0.95f);
    //
    isPlacingCommand = (release_flag) || ((w_H_obj.block<3,1>(0,3)-xDo_placing).norm()<=0.05); // 0.07
    isTossingCommand = (release_flag) || ((w_H_obj.block<3,1>(0,3)-release_position).norm()<=0.05);

    if((isPlacing && isPlacingCommand) || (isThrowing && isTossingCommand)){
      isReleasePositionReached = true;
      dsThrowing.reset_release_flag();
    }
    else if((w_H_obj.block<3,1>(0,3)-w_H_Do.block<3,1>(0,3)).norm()<=0.05){
      isReleasePositionReached = true;
      dsThrowing.reset_release_flag();
    }
    //
    // extract the translation state
    xEE  = 0.5f*(w_H_EE[LEFT].block(0,3,3,1)+w_H_EE[RIGHT].block(0,3,3,1));
    dxEE = 0.5f*(Vd_ee[LEFT].head(3) + Vd_ee[RIGHT].head(3));
    X_next.push_back(xEE-xEE_0);
    dX_next.push_back(dxEE);
    //
    xEE_0 = xEE;



    pred_count++;

  } // while

  //
  int N = X_next.size();
  float v_avg  = 0.0f;
  float LpXd_X = 0.0f;
  std::cout << " PREDICTION HORIZON is : \t " << N << std::endl;
  for(int i=0; i<N; i++){
    v_avg  += dX_next[i].norm()/N;
    LpXd_X += X_next[i].norm();
    // std::cout << " DELTA X_next [" << i << "] : " << X_next[i].norm();
  }
  //
  std::cout << " " << std::endl;
  //
  Eigen::Vector2f Lp_dx_avg = {LpXd_X, v_avg};
  return Lp_dx_avg;

}


void dualArmFreeMotionController::getCoordinatedTranslation(Eigen::Vector3f x_ee[],  
                                                            Eigen::Vector3f x_gp[], 
                                                            Eigen::Vector3f x_std[],
                                                            Eigen::Matrix3f w_R_o, 
                                                            Eigen::Vector3f (&vd_ee)[NB_ROBOTS])
{
  Eigen::Vector3f x_abs_ee  = 0.5f*(x_ee[RIGHT]  + x_ee[LEFT]);
  Eigen::Vector3f x_abs_gp  = 0.5f*(x_gp[RIGHT]  + x_gp[LEFT]);
  Eigen::Vector3f x_abs_stb = 0.5f*(x_std[RIGHT] + x_std[LEFT]);
  //
  Eigen::Vector3f x_rel_ee  = (x_ee[RIGHT]  - x_ee[LEFT]);
  Eigen::Vector3f x_rel_gp  = (x_gp[RIGHT]  - x_gp[LEFT]);
  Eigen::Vector3f x_rel_stb = (x_std[RIGHT] - x_std[LEFT]);
  //
  Eigen::Vector3f x_rel_pgrasp = x_rel_gp;
  x_rel_pgrasp(1) = x_rel_gp(1)/fabs(x_rel_gp(1)) * (fabs(x_rel_gp(1)) + 0.30f);

  Eigen::Vector3f d_p_abs = reachable_p*x_abs_gp + (1.0f-reachable_p)*x_abs_stb;
  // =======================================
  // Absolute velocity of the End-effectors
  // =======================================
  Eigen::Vector3f error_abs = x_abs_ee - d_p_abs;

  // computing the velocity
  // ~~~~~~~~~~~~~~~~~~~~~~~
  Eigen::Vector3f v_abs = -3.0f* gain_p_abs * error_abs;

  Eigen::Vector3f o_error_pos_abs     = w_R_o.transpose() * error_abs;
  Eigen::Vector3f o_error_pos_abs_paral = Eigen::Vector3f(o_error_pos_abs(0), 0.0f, o_error_pos_abs(2));
  float cp_ap   = computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.06f, 1.0f, true);
  float cpl_rel = computeCouplingFactor(error_abs, 50.0f, 0.08f, 1.0f, true);  //

  Eigen::Vector3f d_p_rel   = cpl_rel *(cp_ap * x_rel_gp + (1.0f-cp_ap) *x_rel_pgrasp) + (1.0f-cpl_rel) * x_rel_stb; // TBC 
  Eigen::Vector3f error_rel = x_rel_ee - d_p_rel;
  // =====================================
  // Relative velocity of the hands
  // =====================================

  // computing the velocity
  Eigen::Vector3f v_rel = -5.0f* gain_p_rel * error_rel;
  // ========================================
  // Computation of individual EE motion
  // ========================================
  // velocity
  float a_bi = 0.5f;
  float b_bi = 1.0f;

  vd_ee[LEFT]  = v_abs - (1-a_bi)*v_rel;
  vd_ee[RIGHT] = v_abs +     a_bi*v_rel;
}


Eigen::Vector2f dualArmFreeMotionController::predictRobotTranslation(Eigen::Matrix4f w_H_ee[],  
                                                                      Eigen::Matrix4f w_H_gp[], 
                                                                      Eigen::Matrix4f w_H_eeStandby[], 
                                                                      Eigen::Matrix4f w_H_o,
                                                                      Eigen::Vector3f x_release,
                                                                      float vtoss,
                                                                      float tolerance_dist2contact,
                                                                      float dt,
                                                                      float speedScaling)
{

  Eigen::Vector3f x_ee[NB_ROBOTS], x_gp[NB_ROBOTS], x_std[NB_ROBOTS], x_obj;

  x_ee[LEFT]  = w_H_ee[LEFT].block(0,3,3,1);
  x_ee[RIGHT] = w_H_ee[RIGHT].block(0,3,3,1);
  x_gp[LEFT]  = w_H_gp[LEFT].block(0,3,3,1);
  x_gp[RIGHT] = w_H_gp[RIGHT].block(0,3,3,1);
  x_std[LEFT] = w_H_eeStandby[LEFT].block(0,3,3,1);
  x_std[RIGHT]= w_H_eeStandby[RIGHT].block(0,3,3,1);
  x_obj       = w_H_o.block<3,1>(0,3);
  Eigen::Matrix3f w_R_o = w_H_o.block<3,3>(0,0);
  //
  Eigen::Vector3f vd_ee[NB_ROBOTS], vd_o;

  bool isTossingCommand = false;
  bool isContact = false;
  bool isReleasePositionReached = false;
  float tol_dist2contact = tolerance_dist2contact;
  float a_bi = 0.5f;
  float b_bi = 1.0f;

  int max_horizon = 30;
  int pred_count  = 0;

  //
  std::vector<Eigen::Vector3f> X_next_l, X_next_r;
  std::vector<Eigen::Vector3f> dX_next_l, dX_next_r;
  Eigen::Vector3f xEE_l,  xEE_0_l, dxEE_l; 
  Eigen::Vector3f xEE_r,  xEE_0_r, dxEE_r; 

  xEE_0_l  = x_ee[LEFT]; 
  xEE_0_r  = x_ee[RIGHT]; 

  // robot path length and 
  Eigen::Vector2f Lp_dx_avg = {1e-4, 1e-4};



  while((!isReleasePositionReached) && (pred_count<max_horizon))
  {
    // update contact status
    // Positioning error
    Eigen::Vector3f lh_er = (x_gp[LEFT] -x_ee[LEFT]);   // 
    Eigen::Vector3f rh_er = (x_gp[RIGHT]-x_ee[RIGHT]);  // 
    // Distances to contacts
    if((lh_er.norm() <= tol_dist2contact) && (rh_er.norm() <= tol_dist2contact)){ 
      isContact = true;
    }
    else{
      isContact = false;
    }

    // --------------------------------------------------------------------
    // --------------------------------------------------------------------
    if(!isContact){
      this->getCoordinatedTranslation( x_ee, x_gp, x_std, w_R_o, vd_ee);
      vd_o.setZero();
    }
    else{
      vd_o = vtoss * (x_release - x_obj).normalized();

      Eigen::Vector3f v_abs = vd_o;
      Eigen::Vector3f v_rel = Eigen::Vector3f(0,0,0);

      vd_ee[LEFT]  = v_abs - (1-a_bi)*v_rel;
      vd_ee[RIGHT] = v_abs +     a_bi*v_rel;
    }
    // --------------------------------------------------------------------
    // --------------------------------------------------------------------
    vd_ee[LEFT]  *= speedScaling;
    vd_ee[RIGHT] *= speedScaling;
    vd_o *= speedScaling;
    // ---------------------------
    // update position from velocity
    x_ee[LEFT]   = x_ee[LEFT]  + dt * vd_ee[LEFT];
    x_ee[RIGHT]  = x_ee[RIGHT] + dt * vd_ee[RIGHT];
    x_gp[LEFT]   = x_gp[LEFT]  + dt * vd_o;
    x_gp[RIGHT]  = x_gp[RIGHT] + dt * vd_o;
    x_obj        = x_obj + dt * vd_o;

    // update Convergence status
    // isPlacingCommand = (release_flag) || ((w_H_obj.block<3,1>(0,3)-xDo_placing).norm()<=0.05); // 0.07
    isTossingCommand = ((x_obj-x_release).norm()<=0.05);

    if((isTossingCommand)){
      isReleasePositionReached = true;
    }
    //
    // extract the translation state
    //--------------------------------
    // xEE  = 0.5f*(x_ee[LEFT]  + x_ee[RIGHT]);
    // dxEE = 0.5f*(vd_ee[LEFT] + vd_ee[RIGHT]);
    //
    X_next_l.push_back(x_ee[LEFT]-xEE_0_l);
    X_next_r.push_back(x_ee[RIGHT]-xEE_0_r);
    dX_next_l.push_back(vd_ee[LEFT]);
    dX_next_r.push_back(vd_ee[RIGHT]);
    //
    xEE_0_l = x_ee[LEFT];
    xEE_0_r = x_ee[RIGHT];

    pred_count++;
  } // while

  //
  int N = X_next_l.size();
  float v_avg  = 0.0f;
  float LpXd_X = 0.0f;
  std::cout << " PREDICTION HORIZON is : \t " << N << std::endl;
  for(int i=0; i<N; i++){
    v_avg  += 0.5f*(dX_next_l[i].norm() + dX_next_r[i].norm())/N;
    LpXd_X += 0.5f*(X_next_l[i].norm()  + X_next_r[i].norm());
    // std::cout << " dX_next_l [" << i << "] : " << dX_next_l[i].norm();
  }
  //
  std::cout << " " << std::endl;
  //
  Lp_dx_avg << LpXd_X, v_avg;


  return Lp_dx_avg;
}