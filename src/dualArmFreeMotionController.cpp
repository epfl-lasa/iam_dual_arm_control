
#include "dualArmFreeMotionController.h"
#include "Utils.hpp"


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
  _refVreach    = 0.0f;

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
  _v_max = 2.0f;     // velocity limits
  _w_max = 4.0f;     // velocity limits

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
  // get the task transformations
  Eigen::Matrix4f w_H_ar, lr_H_rr;           // absolute and relative EE poses
  Eigen::Matrix4f w_H_ap, lp_H_rp;           // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_ar_stb, lr_H_rr_stb;   // absolute and relative EE standby poses
  Eigen::Matrix4f lp_H_rp_pgrasp;            // relative pregrasp EE pose
  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(w_H_ee[LEFT], w_H_ee[RIGHT], w_H_ar, lr_H_rr);    // EE
  Utils<float>::getBimanualTransforms(w_H_gp[LEFT], w_H_gp[RIGHT], w_H_ap, lp_H_rp);      // object's grasp points
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
  float coord_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.01f, 1.0f, true);
  // Coupling the orientation with the position error
  Eigen::Matrix3f d_R_rel = reachable_p* (coord_abs*lp_H_rp.block<3,3>(0,0) + (1.0f-coord_abs)*lp_H_rp_pgrasp.block<3,3>(0,0)) 
                          + (1.0f- reachable_p)*lr_H_rr_stb.block<3,3>(0,0);

  Eigen::Matrix4f lr_H_rr_t = lr_H_rr;
  lr_H_rr_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(coord_abs, lr_H_rr.block<3,3>(0,0), d_R_rel); //desired
  // relative transformation
  // Eigen::Matrix4f d_H_c_rel = lr_H_rr_t.inverse() * lr_H_rr;  // expressed in the left hand frame 

  // ///////////////////////////////////////////////////////////////////////////////////////////
  // Computation of desired orientation
  this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);

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
  // get the task transformations
  Eigen::Matrix4f w_H_ar, lr_H_rr;           // absolute and relative EE poses
  Eigen::Matrix4f w_H_ap, lp_H_rp;           // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_ar_stb, lr_H_rr_stb;   // absolute and relative EE standby poses
  Eigen::Matrix4f lp_H_rp_pgrasp;            // relative pregrasp EE pose
  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(w_H_ee[LEFT], w_H_ee[RIGHT], w_H_ar, lr_H_rr);    // EE
  Utils<float>::getBimanualTransforms(w_H_gp[LEFT], w_H_gp[RIGHT], w_H_ap, lp_H_rp);      // object's grasp points
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
  this->computeDesiredOrientation(coord_abs, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);

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
    Eigen::Vector3f d_p_ee     = reachable_p *w_H_gp[k].block<3,1>(0,3) + (1.0f-reachable_p)*w_H_ee[k].block<3,1>(0,3);
      Eigen::Matrix3f d_R_ee   = reachable_p *w_H_gp[k].block<3,3>(0,0) + (1.0f-reachable_p)*w_H_ee[k].block<3,3>(0,0);
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
  // get the task transformations
  Eigen::Matrix4f w_H_ar, lr_H_rr;           // absolute and relative EE poses
  Eigen::Matrix4f w_H_ap, lp_H_rp;           // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_ar_stb, lr_H_rr_stb;   // absolute and relative EE standby poses
  Eigen::Matrix4f lp_H_rp_pgrasp;            // relative pregrasp EE pose
  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(w_H_ee[LEFT], w_H_ee[RIGHT], w_H_ar, lr_H_rr);    // EE
  Utils<float>::getBimanualTransforms(w_H_gp[LEFT], w_H_gp[RIGHT], w_H_ap, lp_H_rp);      // object's grasp points
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
  this->computeDesiredOrientation(1.0f, w_H_ee, _w_H_eeStandby, w_H_o, qd, isOrient3d);

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
  // 
  Eigen::Matrix4f w_H_o_z, w_H_Do_z;   // current and desired object pose but with height of via plane
  Eigen::Matrix4f w_H_ap, lp_H_rp;     // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_ar, lr_H_rr;     // absolute and relative EE poses

  w_H_o_z       = w_H_o;
  w_H_Do_z      = w_H_Do;
  w_H_o_z(2,3)  = w_H_Do(2,3) + via_height;
  w_H_Do_z(2,3) = w_H_Do(2,3) + via_height;
  // Bimanual transformation
  Utils<float>::getBimanualTransforms(w_H_gp[LEFT], w_H_gp[RIGHT], w_H_ap, lp_H_rp);    // object's grasp points
  Utils<float>::getBimanualTransforms(w_H_ee[LEFT], w_H_ee[RIGHT], w_H_ar, lr_H_rr);    // EE

  Eigen::Vector3f error_z  = Eigen::Vector3f(0.f, 0.f, w_H_o(2,3) - w_H_o_z(2,3));
  Eigen::Vector3f error_xy = Eigen::Vector3f(w_H_o(0,3)-w_H_Do_z(0,3), w_H_o(1,3)-w_H_Do_z(1,3), 0.0f); //w_H_o.block<2,1>(0,3) - w_H_Do_z.block<2,1>(0,3); 

  float cpl_oz   = Utils<float>::computeCouplingFactor(error_z, 50.0f, 0.12f, 1.2f, true);
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
  this->computeDesiredOrientation(1.0f, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);
  
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
  // get the task transformations
  Eigen::Matrix4f w_H_ar, lr_H_rr;           // absolute and relative EE poses
  Eigen::Matrix4f w_H_ap, lp_H_rp;           // absolute and relative object's grasp points
  Eigen::Matrix4f w_H_ar_stb, lr_H_rr_stb;   // absolute and relative EE standby poses
  Eigen::Matrix4f lp_H_rp_pgrasp;            // relative pregrasp EE pose
  // Bimanual coordinated task-space transforms
  Utils<float>::getBimanualTransforms(w_H_ee[LEFT], w_H_ee[RIGHT], w_H_ar, lr_H_rr);    // EE
  Utils<float>::getBimanualTransforms(w_H_gp[LEFT], w_H_gp[RIGHT], w_H_ap, lp_H_rp);      // object's grasp points
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
  _V_abs.head(3) = -5.0f* gain_p_abs * _error_abs.head(3);
  _V_abs.tail(3) = -5.0f* jacMuTheta_abs.inverse() * gain_o_abs * _error_abs.tail(3);

  // =====================================
  // Relative velocity of the hands
  // =====================================
  float coord_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.01f, 1.0f, true);
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
  this->computeDesiredOrientation(1.0f, w_H_ee, w_H_gp, w_H_o, qd, isOrient3d);

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
  a_proximity_  *= (1.0f-a_retract_);
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
  release_flag = release_flag_;

  float activation   = a_proximity_;
  // activation = 0.0f;


  // state-dependent gain matrix
  // ----------------------------
    Eigen::Matrix4f w_H_gp_t[NB_ROBOTS];  // <----------------------
    w_H_gp_t[LEFT].block<3,3>(0,0) = w_H_gp[LEFT].block<3,3>(0,0);
    w_H_gp_t[LEFT].block<3,1>(0,3) = Xb[LEFT];
    //
    w_H_gp_t[RIGHT].block<3,3>(0,0) = w_H_gp[RIGHT].block<3,3>(0,0);
    w_H_gp_t[RIGHT].block<3,1>(0,3) = Xb[RIGHT];

    std::cout << " POSITION : ---------- [w_H_gp_t] \t" << w_H_gp_t[LEFT].block<3,1>(0,3).transpose() << " and \t" << w_H_gp_t[RIGHT].block<3,1>(0,3).transpose() << std::endl;
    Vector6f Vd_ee_nom[NB_ROBOTS];
    Eigen::Vector4f qd_nom[NB_ROBOTS];
    //
    this->computeCoordinatedMotion2(w_H_ee,  w_H_gp_t, w_H_o, Vd_ee_nom, qd_nom, isOrient3d);
    //
    Vector6f DS_ee_nominal = Eigen::VectorXf::Zero(6);
    DS_ee_nominal.head(3)  = Vd_ee_nom[LEFT].head(3);
    DS_ee_nominal.tail(3)  = Vd_ee_nom[RIGHT].head(3);

    // float v_reach = (1.f - a_proximity_)*0.7f;
    float v_reach = sqrt(2.0f)*0.6f;
  
    Eigen::Vector3f error_p_abs     = w_H_o.block(0,3,3,1) - 0.5f*( w_H_ee[LEFT].block(0,3,3,1) +  w_H_ee[RIGHT].block(0,3,3,1));
    Eigen::Vector3f o_error_pos_abs = w_H_o.block<3,3>(0,0).transpose() * error_p_abs;
    Eigen::Vector3f o_error_pos_abs_paral = Eigen::Vector3f(o_error_pos_abs(0), 0.0f, o_error_pos_abs(2));
    float cp_ap = Utils<float>::computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.08f, 1.0f, true);  // 50.0f, 0.05f, 2.8f /  50.0f, 0.15f, 1.0f
    float  alp = 0.05f;
    _refVreach = (1.0f-alp)*_refVreach + alp*((1.0f-cp_ap)*_desVreach + cp_ap* VdImp[LEFT].norm());
    //
    // DS_ee_nominal.head(3)  = DS_ee_nominal.head(3)/(DS_ee_nominal.head(3).norm()+1e-10) *  _refVreach;
    // DS_ee_nominal.tail(3)  = DS_ee_nominal.tail(3)/(DS_ee_nominal.tail(3).norm()+1e-10) *  _refVreach;


    //
    Matrix6f A = Eigen::MatrixXf::Identity(6,6);
    A.block<3,3>(0,0) = -3.0f * this->gain_p_abs;
    A.block<3,3>(3,3) = -3.0f * this->gain_p_rel;
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
      case 1: {   // reaching with impact
        Xstar_dual = (1.0 - a_normal_) * Xb_dual + a_normal_ * (X_dual - A_prime.inverse()*VdImp_dual);  // TO DO: add 
        if(VdImp[LEFT].norm() <= 0.01f || VdImp[RIGHT].norm() <= 0.01f){
          Xstar_dual = (1.0 - a_normal_) * Xb_dual + a_normal_ * Xdes_dual;  // TO DO: add 
        }
        //
        Amodul_ee_norm = A_prime*(X_dual - Xb_dual);          // Modulated DS that aligned  the EE with the desired velocity
        Amodul_ee_tang = A_prime*(X_dual - Xstar_dual);  // ;
      }
      break;
      case 2:{ // point to point motion of the object 
        Vector6f X_bi = Eigen::VectorXf::Zero(6);
        X_bi.head(3)  = w_H_Do.block<3,1>(0,3) - w_H_o.block<3,1>(0,3)+ 0.5f*(X[LEFT] + X[RIGHT]);
        X_bi.tail(3)  = 0.99f*(X[RIGHT] - X[LEFT]);

        Xstar_dual =  _Tbi.inverse() * X_bi;
        //
        activation = 1.0f;
        // a_normal_  = 1.0f;
        // a_tangent_ = 0.0f;
        Amodul_ee_norm = _Tbi.inverse() * A * _Tbi *(X_dual - Xstar_dual);          // Modulated DS that aligned  the EE with the desired velocity
        Amodul_ee_tang = _Tbi.inverse() * A * _Tbi *(X_dual - Xstar_dual);  // ;
      }
      break;

      case 3:{ //  velocity based motion of the object
        Vector6f X_bi = Eigen::VectorXf::Zero(6);
        X_bi.head(3)  = 0.5f*(X[LEFT] + X[RIGHT]);
        X_bi.tail(3)  = 0.95f*(X[RIGHT] - X[LEFT]);
        Xstar_dual =  _Tbi.inverse() * X_bi;
        //velocity based motion of the object
        Vector6f Xdot_bi = Eigen::VectorXf::Zero(6);
        Eigen::Vector3f X_rel = X[RIGHT] - X[LEFT];
        Eigen::Vector3f w_o   = Vd_o.tail(3);

        Xdot_bi.head(3)  = Vd_o.head(3);
        Xdot_bi.tail(3)  = w_o.cross(X_rel);
        Amodul_ee_norm = _Tbi.inverse() * ( Xdot_bi +  A * _Tbi*(X_dual - Xstar_dual) );          // Modulated DS that aligned  the EE with the desired velocity
        Amodul_ee_tang = _Tbi.inverse() * ( Xdot_bi +  A * _Tbi*(X_dual - Xstar_dual) );          // ;  
        //
        //
        // Amodul_ee_norm.head(3) = Amodul_ee_norm.head(3)/( Amodul_ee_norm.head(3).norm() +1e-10) * Vd_o.head(3).norm();
        // Amodul_ee_norm.tail(3) = Amodul_ee_norm.tail(3)/( Amodul_ee_norm.tail(3).norm() +1e-10) * Vd_o.head(3).norm();
        // Amodul_ee_tang.head(3) = Amodul_ee_tang.head(3)/( Amodul_ee_tang.head(3).norm() +1e-10) * Vd_o.head(3).norm();
        // Amodul_ee_tang.tail(3) = Amodul_ee_tang.tail(3)/( Amodul_ee_tang.tail(3).norm() +1e-10) * Vd_o.head(3).norm();
        //
        activation = 1.0f;
      }
      break;

      case 4:{ // point to point motion of the object 
        Vector6f X_bi = Eigen::VectorXf::Zero(6);
        X_bi.head(3)  = (1.0f-1.0f*sw_norm_Do)*(w_H_Do.block<3,1>(0,3) - w_H_o.block<3,1>(0,3))+ 0.5f*(X[LEFT] + X[RIGHT]);
        X_bi.tail(3)  = (1.0f-1.0f*sw_norm_Do)*0.95f*(X[RIGHT] - X[LEFT]) + 1.0f*sw_norm_Do*0.95f*(X[RIGHT] - X[LEFT]);
        Xstar_dual    =  _Tbi.inverse() * X_bi;
        //
        //velocity based motion of the object
        Vector6f Xdot_bi      = Eigen::VectorXf::Zero(6);
        Eigen::Vector3f X_rel = X[RIGHT] - X[LEFT];
        Eigen::Vector3f w_o   = Vd_o.tail(3);
        Xdot_bi.head(3)       = Vd_o.head(3);
        Xdot_bi.tail(3)       = w_o.cross(X_rel);

        // Amodul_ee_norm = A_prime*(X_dual - Xstar_dual) + sw_norm_Do*_Tbi.inverse() * Xdot_bi;  // Modulated DS that aligned  the EE with the desired velocity
        // Amodul_ee_tang = A_prime*(X_dual - Xstar_dual) + sw_norm_Do*_Tbi.inverse() * Xdot_bi;  // ;
        
        Vector6f v_task_bi = ( A * _Tbi*(X_dual - Xstar_dual) + 1.0f*sw_norm_Do* Xdot_bi );
        // v_task_bi.head(3) = v_task_bi.head(3)/(v_task_bi.head(3).norm()+1e-10)*1.0f*v_task_bi.head(3).norm();
        v_task_bi.head(3) = v_task_bi.head(3)/(v_task_bi.head(3).norm()+1e-10)*Vd_o.head(3).norm();

        Amodul_ee_norm = _Tbi.inverse() * v_task_bi;  // 
        Amodul_ee_tang = _Tbi.inverse() * v_task_bi;  //
        //
        // Amodul_ee_norm.head(3) = Amodul_ee_norm.head(3)/( Amodul_ee_norm.head(3).norm() +1e-10) * Vd_o.head(3).norm();
        // Amodul_ee_norm.tail(3) = Amodul_ee_norm.tail(3)/( Amodul_ee_norm.tail(3).norm() +1e-10) * Vd_o.head(3).norm();
        // Amodul_ee_tang.head(3) = Amodul_ee_tang.head(3)/( Amodul_ee_tang.head(3).norm() +1e-10) * Vd_o.head(3).norm();
        // Amodul_ee_tang.tail(3) = Amodul_ee_tang.tail(3)/( Amodul_ee_tang.tail(3).norm() +1e-10) * Vd_o.head(3).norm();
        //
        activation = 1.0f;
        // a_normal_  = 1.0f;
        // a_tangent_ = 0.0f;
      }
      break;

    }

    // Vector6f Amodul_ee_norm = _Tbi.inverse() * ;          // Modulated DS that aligned  the EE with the desired velocity
    // Vector6f Amodul_ee_tang = _Tbi.inverse() * ;  // ;
    
  // // get the modulated motion (out_motion: Velocity)
    Vector6f DS_ee_modulated = Eigen::VectorXf::Zero(6,1);
    DS_ee_modulated = this->compute_modulated_motion_dual(activation, BasisQ, DS_ee_nominal, Amodul_ee_norm, Amodul_ee_tang);
  // // }
    // DS_ee_modulated.head(3)  = DS_ee_modulated.head(3)/(DS_ee_modulated.head(3).norm()+1e-10) *  _refVreach;
    // DS_ee_modulated.tail(3)  = DS_ee_modulated.tail(3)/(DS_ee_modulated.tail(3).norm()+1e-10) *  _refVreach;


  //   //
    Vd_ee[LEFT].head(3) = DS_ee_modulated.head(3);
    // Vd_ee[LEFT].head(3) = DS_ee_nominal.head(3);
    Vd_ee[LEFT].tail(3) = Vd_ee_nom[LEFT].tail(3);
    //
    Vd_ee[RIGHT].head(3) = DS_ee_modulated.tail(3);
    // Vd_ee[RIGHT].head(3) = DS_ee_nominal.tail(3);
    Vd_ee[RIGHT].tail(3) = Vd_ee_nom[RIGHT].tail(3);

    //
    qd[LEFT]  = qd_nom[LEFT];
    qd[RIGHT] = qd_nom[RIGHT];
  
}