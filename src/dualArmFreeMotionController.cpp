
#include "dualArmFreeMotionController.h"
#include "Utils.hpp"


float computeCouplingFactor(Eigen::Vector3f ep_, float alpha_, float beta_, float gamma_, bool secondOrder)
{
	float t_cpl_ = 1.0f/(alpha_*ep_.norm()+1e-15f);         
	float cpl_   = 0.0f;
	t_cpl_ = pow(t_cpl_,gamma_);
	if(secondOrder)	cpl_   = 1.0f - exp(-t_cpl_/beta_) *(1.0f + t_cpl_/beta_);  // 2nd order critically damped
	else						cpl_   = 1.0f - exp(-t_cpl_/beta_); 												// 1st order increase

	return cpl_;
}

Eigen::Matrix<float,4,1> updateOrientationQuat(Eigen::Matrix<float,4,1> qin, Eigen::Matrix<float,3,1> win)
{
  Eigen::Matrix<float,4,1> qout, q0, q1, w0;
  w0.setZero();
  w0.segment(0,3) = win;
  q0.segment(0,3) = qin.segment(1,3);
  q0(3) = qin(0);
  Eigen::Matrix<float,4,1> dqe =  Utils<float>::quat_exp(0.5f * w0);
  q1 = Utils<float>::quat_mul(dqe, q0);
  qout.segment(1,3) = q1.segment(0,3);
  qout(0) = q1(3);
  return qout;
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

}

dualArmFreeMotionController::~dualArmFreeMotionController(){}

// publishing of the reference trajectories
bool dualArmFreeMotionController::init(Eigen::Matrix4f w_H_eeStandby[], Matrix6f gain_abs_, Matrix6f gain_rel_)
{
	//
	memcpy(_w_H_eeStandby, &w_H_eeStandby[0], NB_ROBOTS * sizeof * w_H_eeStandby);
	//
	reachable_p = 1.0f;
	_v_max 			= 0.7f;
	_w_max 			= 2.0f;
  // _v_max = 1.0f;     // velocity limits
  // _w_max = 4.0f;     // velocity limits

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
  lp_H_rp_pgrasp(1, 3) = lp_H_rp(1,3)/fabs(lp_H_rp(1,3)) * (fabs(lp_H_rp(1,3)) + 0.38f);

  // =======================================
  // Absolute velocity of the End-effectors
  // =======================================
  Eigen::Vector3f d_p_abs = reachable_p *w_H_ap.block<3,1>(0,3) + (1.0f-reachable_p)*w_H_ar_stb.block<3,1>(0,3);
  _error_abs.head(3)      = w_H_ar.block<3,1>(0,3) - d_p_abs;
  std::cout << "[dual_arm_control]: Absolute Error: \t" <<  _V_abs.transpose() << std::endl;

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

  std::cout << "[dual_arm_control]: Absolute Velo: \t" <<  _V_abs.transpose() << std::endl;

  // =====================================
  // Relative velocity of the hands
  // =====================================
  float coord_abs = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.02f, 1.0f, true);
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
  Eigen::Matrix3f jacMuTheta_rel = Utils<float>::getMuThetaJacobian(d_H_c_rel.block<3,3>(0,0)) * w_H_ee[LEFT].block<3,3>(0,0).transpose(); // wrt. the world

  // ///////////////////////////////////////////////////////////////////////////////////////
  float cpl_rel    = computeCouplingFactor(_error_abs.head(3), 50.0f, 0.08f, 1.0f, true);  // 50.0f, 0.05f, 2.8f  0.5
  // float cpl_rel    = computeCouplingFactor(_error_rel.tail(3), 50.0f, 0.1f, 1.0f, false);  // 50.0f, 0.05f, 2.8f

  Eigen::Vector3f o_error_pos_abs = w_H_o.block<3,3>(0,0).transpose() * _error_abs.head(3);
  Eigen::Vector3f o_error_pos_abs_paral = Eigen::Vector3f(o_error_pos_abs(0), 0.0f, o_error_pos_abs(2));
  float cp_ap = computeCouplingFactor(o_error_pos_abs_paral, 50.0f, 0.15f, 1.0f, true);  // 50.0f, 0.05f, 2.8f


  // position error accounting for the reachability of the target
  // Eigen::Vector3f d_p_rel = reachable_p *(cpl_rel*cp_ap* lp_H_rp.block<3,1>(0,3) + (1.0f-cpl_rel*cp_ap) *lp_H_rp_pgrasp.block<3,1>(0,3)) + (1.0f-reachable_p) * lr_H_rr_stb.block<3,1>(0,3); // TBC 
  Eigen::Vector3f d_p_rel = cpl_rel *(cp_ap * lp_H_rp.block<3,1>(0,3) + (1.0f-cp_ap) *lp_H_rp_pgrasp.block<3,1>(0,3)) + (1.0f-cpl_rel) * lr_H_rr_stb.block<3,1>(0,3); // TBC 

  _error_rel.head(3) = lr_H_rr.block<3,1>(0,3) - d_p_rel;  // 

  // computing the velocity
  _V_rel.head(3) = -gain_p_rel * _error_rel.head(3);
  _V_rel.tail(3) = -jacMuTheta_rel.inverse() * gain_o_rel * _error_rel.tail(3);

  std::cout << "[dual_arm_control]: Relative Velo: \t" <<  _V_rel.transpose() << std::endl;

  // ========================================
  // Computation of individual EE motion
  // ========================================
  // velocity
  float a_bi = 0.5f;
  float b_bi = 1.0f;
  Utils<float>::getBimanualTwistDistribution(a_bi, b_bi, _V_abs, _V_rel, Vd_ee[LEFT], Vd_ee[RIGHT]);

  Vd_ee[LEFT]  = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[LEFT]);
  Vd_ee[RIGHT] = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[RIGHT]);

  std::cout << "[dual_arm_control]: CCCCCCCCCCC coord_abs: \t" <<  coord_abs << std::endl;
  // std::cout << "[dual_arm_control]: CCCCCCCCCCC TANH: \t" <<  1.0f-std::tanh(3.0f*_error_rel.head(3).norm()) << std::endl;
  std::cout << "[dual_arm_control]: CCCCCCCCCCC cpl_rel: \t" <<  cpl_rel << std::endl;

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
  _V_abs.head(3) = -gain_p_abs * _error_abs.head(3);
  _V_abs.tail(3) = -jacMuTheta_abs.inverse() * gain_o_abs * _error_abs.tail(3);

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
  Eigen::Matrix3f jacMuTheta_rel = Utils<float>::getMuThetaJacobian(d_H_c_rel.block<3,3>(0,0)) * w_H_ee[LEFT].block<3,3>(0,0).transpose(); // wrt. the world

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
		Vector6f error_ee;	error_ee.setZero();
		Eigen::Vector3f d_p_ee     = reachable_p *w_H_gp[k].block<3,1>(0,3) + (1.0f-reachable_p)*w_H_ee[k].block<3,1>(0,3);
	    Eigen::Matrix3f d_R_ee   = reachable_p *w_H_gp[k].block<3,3>(0,0) + (1.0f-reachable_p)*w_H_ee[k].block<3,3>(0,0);
	    Eigen::Matrix4f w_H_ee_t = w_H_ee[k];
	    w_H_ee_t.block<3,3>(0,0) = Utils<float>::getCombinedRotationMatrix(1.0f, w_H_ee[k].block<3,3>(0,0), d_R_ee); //desired
	    // relative transformation between desired and current frame
	    Eigen::Matrix4f d_H_c_ee = w_H_ee_t.inverse() * w_H_ee[k];
	    error_ee.head(3)      	 = w_H_ee[k].block<3,1>(0,3) - d_p_ee;
	    error_ee.tail(3)      	 = Utils<float>::getPoseErrorCur2Des(d_H_c_ee).tail(3);
	    // 3D Orientation Jacobian 
	  	Eigen::Matrix3f jacMuTheta_ee = Utils<float>::getMuThetaJacobian(d_H_c_ee.block<3,3>(0,0)) * w_H_ee[k].block<3,3>(0,0).transpose();
	  	// ---------------------------------
	  	// computing of desired ee velocity
	  	// ---------------------------------
		Vd_ee[k].head(3) = -gain_p_abs * error_ee.head(3);
	 	Vd_ee[k].tail(3) = -jacMuTheta_ee.inverse() * gain_o_abs * error_ee.tail(3);
	 	Vd_ee[k]  		   = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[k]);
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
      // Eigen::Matrix3f gpRo    = w_H_gp[k].block(0,0, 3,3).transpose() * w_H_o.block(0,0, 3,3);
    //   std::cout << "[dual_arm_control]: RRRRRRRR BEFORE gpRo[" << k << "]: \n"  << gpRo << std::endl;
    //   Eigen::Vector3f ang_rpy = Utils<float>::getEulerAnglesXYZ_FixedFrame(gpRo);
    //                   gpRo    = Utils<float>::eulerAnglesToRotationMatrix(0.0f, 0.0f, ang_rpy(2));
    //   Eigen::Matrix3f wRed    = w_H_o.block(0,0, 3,3) * gpRo.transpose();
    // std::cout << "[dual_arm_control]: RRRRRRRR AFTER wRgd[" << k << "]: \n"  << wRgd << std::endl;
      // std::cout << "[dual_arm_control]: RRRRRRRR wRed[" << k << "]: \n"  << wRed << std::endl;
      // qd[k]  = Utils<float>::getSlerpInterpolation(weight, w_H_ee[k].block(0,0, 3,3), wRed);

      // Eigen::Vector3f zgd  = w_H_gp[k].block(0,0, 3,3).col(2).normalized();
      // Eigen::Vector3f zee  = w_H_ee[k].block(0,0, 3,3).col(2);
      // Eigen::Vector3f xgd  = zgd.cross(zee.normalized()) + Eigen::Vector3f(1e-8f, 0.0f, 0.0f);
      // Eigen::Vector3f ygd  = zgd.cross(xgd);
      // Eigen::Matrix3f wRgd; wRgd << xgd(0),ygd(0),zgd(0), 
      //                               xgd(1),ygd(1),zgd(1), 
      //                               xgd(2),ygd(2),zgd(2);
      // qd[k]  = Utils<float>::getSlerpInterpolation(weight, w_H_ee[k].block(0,0, 3,3), wRgd);

      qd[k]  = Utils<float>::getSlerpInterpolation(weight, w_H_ee[k].block(0,0, 3,3), w_H_gp[k].block(0,0, 3,3));
    }
  }
  else
  {
    for(int k = 0; k < NB_ROBOTS; k++)
    {
      Eigen::Vector3f ref;
      // if(k == (int) RIGHT)
      // {
      //   ref = -_xdD.normalized();
      // }
      // else
      // {
      //   ref = _xdD.normalized();
      // }
      ref = w_H_gp[k].block<3,1>(0,2);
      ref.normalize();

      // Compute rotation error between current orientation and plane orientation using Rodrigues' law
      Eigen::Vector3f u;

      // u = (_wRb[k].col(2)).cross(ref);
      // float c = (_wRb[k].col(2)).transpose()*ref;  
      u = (w_H_ee[k].block<3,3>(0,0).col(2)).cross(ref);
      float c = (w_H_ee[k].block<3,3>(0,0).col(2)).transpose()*ref;  
      float s = u.norm();
      u /= s;
      
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

      // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the object surface
      // _qd[k] = Utils<float>::slerpQuaternion(q_[k],qf,1.0f-std::tanh(3.0f*_eD)); // _error_rel.head(3)
      Eigen::Vector3f error_pos = w_H_ee[k].block(0,3, 3,1) - w_H_gp[k].block(0,3, 3,1);
      // qd[k] = Utils<float>::slerpQuaternion(q_,qf,1.0f-std::tanh(3.0f*error_pos.head(3).norm())); // _error_rel.head(3).norm()
      qd[k] = Utils<float>::slerpQuaternion(q_,qf,1.0f-std::tanh(3.0f*_error_rel.head(3).norm())); // _error_rel.head(3).norm()

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
  Eigen::Matrix3f jacMuTheta_rel = Utils<float>::getMuThetaJacobian(d_H_c_rel.block<3,3>(0,0)) * w_H_ee[LEFT].block<3,3>(0,0).transpose(); // wrt. the world

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

  // float cpl_oz   = Utils<float>::computeCouplingFactor(error_z, 50.0f, 0.12f, 1.2f, true);
  // float cpl_Doxy = Utils<float>::computeCouplingFactor(error_xy, 50.0f, 0.10f, 1.0f, false);
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
  Eigen::Matrix3f jacMuTheta_rel = Utils<float>::getMuThetaJacobian(d_H_c_rel.block<3,3>(0,0)) * w_H_ee[LEFT].block<3,3>(0,0).transpose(); //   // 3D Orientation Jacobian wrt. the world
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
    // Eigen::Vector3f t = w_H_ee[i].block<3,1>(0,3) - w_H_o.block<3,1>(0,3);
    Eigen::Vector3f t = w_H_gp[i].block<3,1>(0,3) - w_H_o.block<3,1>(0,3);
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
  Vector6f Vabs_EE_l = GraspMx_obj_EE.block<6,6>(0,0).transpose() * _V_obj;
  Vector6f Vabs_EE_r = GraspMx_obj_EE.block<6,6>(0,6).transpose() * _V_obj;
  Vd_ee[LEFT]   =  Vabs_EE_l - 0.5f* _V_rel;
  Vd_ee[RIGHT]  =  Vabs_EE_r + 0.5f* _V_rel;

  // applying velocity
  // ========================================
  Vd_ee[LEFT]  = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[LEFT]);
  Vd_ee[RIGHT] = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_ee[RIGHT]);

  Eigen::Matrix<float,4,1> ql = Utils<float>::rotationMatrixToQuaternion(w_H_ee[LEFT].block(0,0,3,3));
  Eigen::Matrix<float,4,1> qr = Utils<float>::rotationMatrixToQuaternion(w_H_ee[RIGHT].block(0,0,3,3));

  Eigen::Matrix<float,4,1> qd0l  =  updateOrientationQuat(ql,  0.1f*Vabs_EE_l.tail(3));
  Eigen::Matrix<float,4,1> qd0r  =  updateOrientationQuat(qr,  0.1f*Vabs_EE_r.tail(3));

  // qd[LEFT]  = ql;
  // qd[RIGHT] = qr;

  // qd[LEFT]  = qd0l;
  // qd[RIGHT] = qd0r;

  std::cout << "[dual_arm_control]: CCCCCCCCCCC cpl_oz: \t" <<  cpl_oz << std::endl;
  std::cout << "[dual_arm_control]: CCCCCCCCCCC cpl_Doxy: \t" <<  cpl_Doxy << std::endl;
  std::cout << "[dual_arm_control]: CCCCCCCCCCC qd0l: \t" <<  qd0l.transpose() << std::endl;
  std::cout << "[dual_arm_control]: CCCCCCCCCCC qd0r: \t" <<  qd0r.transpose() << std::endl;

  std::cout << "[dual_arm_control]: CCCCCCCCCCC ql: \t" <<  ql.transpose() << std::endl;
  std::cout << "[dual_arm_control]: CCCCCCCCCCC qr: \t" <<  qr.transpose() << std::endl;
}
