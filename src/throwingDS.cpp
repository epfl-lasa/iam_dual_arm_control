

#include "throwingDS.h"


throwingDS::throwingDS(){
	//
	for(int i=0; i<3; i++){

		Kp_[i] = 5.0 *Eigen::MatrixXf::Identity(3,3);
		Dp_[i] = 2.0 *sqrt(Kp_[i](0,0))*Eigen::MatrixXf::Identity(3,3);
		//
		Ko_[i] = 2.5 *Eigen::MatrixXf::Identity(3,3);
		Do_[i] = 2.0 *sqrt(Ko_[i](0,0))*Eigen::MatrixXf::Identity(3,3);
	}
	//
	rho_		    	= 0.12;
	range_norm_  	= 0.04;
	range_tang_  	= 0.02;

	sw_proxim_   	= 150.0;
	sw_norm_     	= 150.0;
	sw_tang_     	= 150.0;
	//
	a_proximity_ 	= 0.0;
	a_normal_ 		= 0.0;
	a_tangent_ 		= 0.0;
	a_retract_   	= 0.0;
	coupling_ 		= 0.0;
	//
	w_H_de_				= Eigen::MatrixXf::Identity(4,4);
	w_H_re_				= Eigen::MatrixXf::Identity(4,4);
	w_H_po_				= Eigen::MatrixXf::Identity(4,4);
	v_toss_				= Eigen::VectorXf::Zero(3);
	w_toss_				= Eigen::VectorXf::Zero(3);
	BasisQp_			= Eigen::MatrixXf::Identity(3,3);
	BasisQo_			= Eigen::MatrixXf::Identity(3,3);
	V_ee_d_ 			= Eigen::VectorXf::Zero(6);
	A_ee_d_ 			= Eigen::VectorXf::Zero(6);
	//
	release_flag_ = false;
	_refVtoss     = 0.00f;
	a_toss_       = 0.0f;

	_v_max				= 1.15f;
	_w_max				= 4.0f;

	// ==============================================================================================================================
	ds_param_.is2ndOrder = false;
	// Modulation parameters
	ds_param_.modulRegion[0] = 0.20;
	ds_param_.modulRegion[1] = 0.025;
	ds_param_.modulRegion[2] = 0.015;
	//
	// ============================================================================================
	// Gains
	// ============================================================================================
	float T_settling = 1.0f; // [Settling time in second]
	float gn = pow((4.0f/T_settling),2.f);
	// Stiffness
	// ==========
	// Position
	ds_param_.Kp[0] << -gn,    0.00f,    0.00f, 0.00f, -gn,    0.00f, 0.00f, 0.00f, -gn;	// Reach
	ds_param_.Kp[1] << -gn,    0.00f,    0.00f, 0.00f, -gn,    0.00f, 0.00f, 0.00f, -gn;	// Toss
	ds_param_.Kp[2] << -gn,    0.00f,    0.00f, 0.00f, -gn,    0.00f, 0.00f, 0.00f, -gn; // retract
	// orientation
	ds_param_.Ko[0] << -1.5f*gn, 0.00f, 0.00f, 0.00f, -1.5f*gn, 0.00f, 0.00f, 0.00f, -1.5f*gn;	// Reach
	ds_param_.Ko[1] << -1.5f*gn, 0.00f, 0.00f, 0.00f, -1.5f*gn, 0.00f, 0.00f, 0.00f, -1.5f*gn;	// Toss
	ds_param_.Ko[2] << -1.5f*gn, 0.00f, 0.00f, 0.00f, -1.5f*gn, 0.00f, 0.00f, 0.00f, -1.5f*gn;	// retract

	 if(ds_param_.is2ndOrder = false)
	 { 
	 	//
  	// Stiffness : choosen to yield the same settling time as the second order
		// =========
		// Position
		ds_param_.Kp[0] << -1.0f * (-1.0f*ds_param_.Kp[0]).cwiseSqrt();		// Reach
		ds_param_.Kp[1] << -1.0f * (-1.0f*ds_param_.Kp[1]).cwiseSqrt();		// Toss
		ds_param_.Kp[2] << -1.0f * (-1.0f*ds_param_.Kp[2]).cwiseSqrt();		// retract
		// orientation
		ds_param_.Ko[0] << -1.0f * (-1.0f*ds_param_.Ko[0]).cwiseSqrt();		// Reach
		ds_param_.Ko[1] << -1.0f * (-1.0f*ds_param_.Ko[1]).cwiseSqrt();		// Toss
		ds_param_.Ko[2] << -1.0f * (-1.0f*ds_param_.Ko[2]).cwiseSqrt();		// retract
  }

	// Damping
	// ==========
	// chosen to yield critically damped motion
	ds_param_.Dp[0] = -2.0f * (-1.0f*ds_param_.Kp[0]).cwiseSqrt();						
	ds_param_.Dp[1] = -2.0f * (-1.0f*ds_param_.Kp[1]).cwiseSqrt();
	ds_param_.Dp[2] = -2.0f * (-1.0f*ds_param_.Kp[2]).cwiseSqrt();

	ds_param_.Do[0] = -2.0f * (-1.0f*ds_param_.Ko[0]).cwiseSqrt();
	ds_param_.Do[1] = -2.0f * (-1.0f*ds_param_.Ko[1]).cwiseSqrt();
	ds_param_.Do[2] = -2.0f * (-1.0f*ds_param_.Ko[2]).cwiseSqrt();

	// ==============================================================================================================================


}
throwingDS::~throwingDS(){

}

bool throwingDS::init(float modulRegion[], Eigen::Matrix3f Kp[], Eigen::Matrix3f Dp[], Eigen::Matrix3f Ko[], Eigen::Matrix3f Do[], bool is2ndOrder){
	
	// initialize the gains
	memcpy(Kp_, &Kp[0], 3 * sizeof *Kp); 
	memcpy(Dp_, &Dp[0], 3 * sizeof *Dp); 
	memcpy(Ko_, &Ko[0], 3 * sizeof *Ko); 
	memcpy(Do_, &Do[0], 3 * sizeof *Do); 

	is2ndOrder_ = is2ndOrder;
	rho_        = modulRegion[0];
	range_norm_ = modulRegion[1];
	range_tang_ = modulRegion[2];

	return true;
}

bool throwingDS::init(tossDsParam ds_param, Eigen::Vector3f releasePos, Eigen::Vector4f releaseOrient, 
											Eigen::Vector3f releaseLinVel, Eigen::Vector3f releaseAngVel, 
											Eigen::Vector3f restPos, Eigen::Vector4f restOrient)
{
	// initialize the gains
	memcpy(Kp_, &ds_param.Kp[0], 3 * sizeof *ds_param.Kp); 
	memcpy(Dp_, &ds_param.Dp[0], 3 * sizeof *ds_param.Dp); 
	memcpy(Ko_, &ds_param.Ko[0], 3 * sizeof *ds_param.Ko); 
	memcpy(Do_, &ds_param.Do[0], 3 * sizeof *ds_param.Do); 
	//
	is2ndOrder_ = ds_param.is2ndOrder;
	rho_        = ds_param.modulRegion[0];
	range_norm_ = ds_param.modulRegion[1];
	range_tang_ = ds_param.modulRegion[2];
	//
	v_toss_  = releaseLinVel;	
	w_toss_  = releaseAngVel;
	//
	w_H_de_ = Utils<float>::pose2HomoMx(releasePos, releaseOrient);
	w_H_re_ = Utils<float>::pose2HomoMx(restPos, restOrient);
	// 
	BasisQp_ = this->createOrthonormalMatrixFromVector(v_toss_);
	BasisQo_ = this->createOrthonormalMatrixFromVector(w_toss_);

	return true;	
}

Vector6f throwingDS::apply(Eigen::Vector3f curPos, Eigen::Vector4f curOrient, Eigen::Vector3f curLinVel, Eigen::Vector3f curAngVel){
	//
	Eigen::Matrix4f w_H_ce = Utils<float>::pose2HomoMx(curPos, curOrient);
	Vector6f Vee  	= Eigen::VectorXf::Zero(6);
	Vee.head(3) 	= curLinVel; 
	Vee.tail(3) 	= curAngVel; 
	bool release_flag = false;
	//
	Vector6f Vd_obj = this->generate_throwing_motion(w_H_ce,  Vee, w_H_de_, w_H_re_, BasisQp_, v_toss_, release_flag);

	Vd_obj  = Utils<float>::SaturationTwist(_v_max, _w_max, Vd_obj);

	float  alp = 0.05f;
  // _refVtoss = (1.0f-alp)*_refVtoss + alp*Vd_obj.head(3).norm(); //(v_toss_.norm());
  _refVtoss = v_toss_.norm();

	Vd_obj.head(3)  = Vd_obj.head(3)/(Vd_obj.head(3).norm()+1e-10)  *  _refVtoss; // v_toss_.norm();
	std::cout << "[throwingDS]:  ------XXXXXXXXXXXXXXXXXXXXX ------ Vd_obj.head(3)   : \t" <<  Vd_obj.head(3).transpose() << std::endl;
	return Vd_obj;
}

Vector6f throwingDS::generate_throwing_motion(Eigen::Matrix4f w_H_ce,  Vector6f Vee, Eigen::Matrix4f w_H_de, Eigen::Matrix4f w_H_re,  
										 													Eigen::Matrix3f BasisQ, Eigen::Vector3f Vdtoss, bool &release_flag)
{
	// States and desired states
	Eigen::Vector3f X 	  = w_H_ce.block<3,1>(0,3);
	Eigen::Vector3f Xdot  = Vee.head(3);
	Eigen::Vector3f Omega = Vee.tail(3);
	Eigen::Vector3f Xdes  = w_H_de.block<3,1>(0,3);	
	Eigen::Vector3f Xretr = w_H_re.block<3,1>(0,3);	
	Eigen::Vector3f Xb    = Xdes + BasisQ*Eigen::Vector3f(-0.5*this->rho_, 0.0, 0.0);
	Eigen::Vector3f Xe    = Xdes + BasisQ*Eigen::Vector3f( 0.2*this->rho_, 0.0, 0.0);
	Eigen::Vector3f Xc    = Xdes + BasisQ*Eigen::Vector3f(-0.2*this->rho_, 0.0, 0.0);  // w_H_po_
	Eigen::Vector3f Xpick = w_H_po_.block<3,1>(0,3);	
	Eigen::Matrix3f Se1   = Eigen::MatrixXf::Zero(3,3); Se1(0,0) = 1.0f;
	Eigen::Vector3f Xti   = Xdes + BasisQ * Se1 * BasisQ.transpose()*(Xpick - Xdes);
	float den = BasisQ.col(0).transpose()*Eigen::Vector3f(1.0, 0.0, 0.0);
	float num = (Xpick - Xti).transpose() * Eigen::Vector3f(1.0, 0.0, 0.0);
	float d = num/( den+ 1e-15) ;
	// Eigen::Vector3f Xt   = 0.40f*Xti+0.60f*Xdes;
	Eigen::Vector3f Xpe = Xti + BasisQ.col(0)*d;
	float beta = 0.50f;
	Eigen::Vector3f Xt  = (1.0f-beta)*Xpe+ beta*Xdes; //Xpe;
	// Xt(2) = 0.90f*Xdes(2);


	//=======================================================================
	// Modulation term
	//=======================================================================
	Eigen::Vector3f Xqb = BasisQ.transpose()*(X - Xb);
	Eigen::Vector3f Xqe = BasisQ.transpose()*(X - Xe);

	float dist2reach   = (X - Xc).norm();
	float dist2line    = Xqb.tail(2).norm();
	float dist2end     = Xqe.head(1).norm();
	
	a_proximity_	= 0.5*(std::tanh(this->sw_proxim_* (0.5*this->rho_ - dist2reach)) + 1.0 ); 	   		// scalar function of 3D distance  to initial (pre-modulation) position of attractor
	a_proximity_	*= (1.0-a_retract_);
	// a_proximity_ = 1.0;

	a_normal_   	= a_proximity_* 0.5*(std::tanh(this->sw_norm_  * (this->range_norm_ - dist2line)) + 1.0 ); 	// scalar function of distance to the line of direction Vtoss and passing through the release position
	a_tangent_  	= a_proximity_* 0.5*(std::tanh(this->sw_tang_  * (this->range_tang_ - dist2end))  + 1.0 ); 	// scalar function of distance to the stopping position of the throwing task
	coupling_     = exp(-0.5*dist2line/(2.0*range_norm_*range_norm_));
	coupling_ = 1.0;
	
	if(a_tangent_ >= 0.95){ 
		a_retract_   = 1.0;
	}
	else if((a_retract_ == 1.0) && (X - Xretr).norm() <= 0.01){  // tolerance within 1 cm
		// a_retract_    = 0.0;
	}
	// 
	if((X-Xdes).norm() <= 2e-2){  // release if the norm is within 1 cm
		release_flag_ = true;
	}

	release_flag = release_flag_;

	float activation   = a_proximity_;

	std::cout << "[throwingDS]:  -------------XXXXXXXXXXXXXXXXXXXXX ------ Xpick : \t" <<  Xpick.transpose() << std::endl;
	std::cout << "[throwingDS]:  -------------XXXXXXXXXXXXXXXXXXXXX ------ Xb  : \t" <<  Xb.transpose() << std::endl;
	std::cout << "[throwingDS]:  -------------XXXXXXXXXXXXXXXXXXXXX ------ Xti   : \t" <<  Xti.transpose() << std::endl;
	std::cout << "[throwingDS]:  -------------XXXXXXXXXXXXXXXXXXXXX ------ Xt   : \t" <<  Xt.transpose() << std::endl;
	

	// float a_normal_t   	= 0.5*(std::tanh(0.8f*this->sw_norm_  * (1.2f*this->range_norm_ - dist2line)) + 1.0 );  // good
	// float a_normal_t   	= 0.5*(std::tanh(1.2f*this->sw_norm_  * (1.0f*this->range_norm_ - dist2line)) + 1.0 );
	float tol_rad = (X-Xt).transpose() * (X-Xt);
	float a_normal_t   	= 0.5*(std::tanh(1.2f*this->sw_norm_  * (0.8f*this->range_norm_ - tol_rad)) + 1.0 );
	// a_normal_t = 0.0f;
	std::cout << "[throwingDS]:  -------------XXXXXXXXXXXXXXXXXXXXX ------ a_normal_t   : \t" <<  a_normal_t << std::endl;
	std::cout << "[throwingDS]:  -------------XXXXXXXXXXXXXXXXXXXXX ------ Vdtoss   : \t" <<  Vdtoss.transpose() << std::endl;
	if(a_normal_t >=0.90f){
      a_toss_ = 1.0f;
    }
  //
  float sw_toss = (a_normal_t + a_toss_);
  if((a_normal_t + a_toss_) >= 1.0f){
    sw_toss = 1.0f;
  }


	// state-dependent gain matrix
	// ----------------------------
	Vector6f Out_motion = Eigen::VectorXf::Zero(6,1);
	//
	if(this->is2ndOrder_){
		// Eigen::Vector3f Xstar = (1.0 - a_normal_) * Xb + a_normal_ * (X + Kp_[TOSS].inverse()*Dp_[TOSS]*Vdtoss );
		Eigen::Vector3f Xstar = (1.0 - a_normal_) * Xb + a_normal_ * (X + Vdtoss - (Eigen::MatrixXf::Identity(3,3)-Kp_[TOSS].inverse()*Dp_[TOSS] )*Xdot );
		// 
		Eigen::Vector3f Areach_ee      = Dp_[REACH]*Xdot + Kp_[REACH]*(X - Xb); 						// DS for approaching the tossing position
		Eigen::Vector3f Amodul_ee_norm = Dp_[TOSS]*Xdot  + Kp_[TOSS]*(X - Xb); 							// Modulated DS that aligned  the EE with the desired velocity
		Eigen::Vector3f Amodul_ee_tang = Dp_[TOSS]*Xdot  + Kp_[TOSS]*(X - Xstar); 					//this->computeModulatedAcceleration(Km, Dm, X, Xdot, Xstar);
		Eigen::Vector3f Aretrac_ee     = Dp_[RETRACT]*Xdot + Kp_[RETRACT]*(X - Xretr); 			// DS for retracting after the tossing position

		// get the modulated motion (out_motion: Acceleration)
		Out_motion.head(3) = (1.0-a_retract_)*this->compute_modulated_motion(activation, BasisQ, Areach_ee, Amodul_ee_norm, Amodul_ee_tang) + a_retract_ * Aretrac_ee;
		// get angular motion
		Out_motion.tail(3) = (1.0-a_retract_)*this->compute_angular_motion(coupling_, w_H_ce, Omega, w_H_de, Ko_[REACH], Do_[REACH], this->is2ndOrder_) 
													+   a_retract_ *this->compute_angular_motion(coupling_, w_H_ce, Omega, w_H_re, Ko_[RETRACT], Do_[RETRACT], this->is2ndOrder_);
	}
	else{
		Eigen::Vector3f Xstar = (1.0 - a_normal_) * Xb + a_normal_ * (X - Kp_[TOSS].inverse()*Vdtoss);
		// 
		// Eigen::Vector3f Areach_ee      = Kp_[REACH]*(X - Xb); 					// DS for approaching the tossing position 
		Eigen::Vector3f Areach_ee      = Kp_[REACH]*(X - (a_normal_t *Xb + (1.0f-a_normal_t)*Xt) ); 					// DS for approaching the tossing position 
		// Eigen::Vector3f Areach_ee      = Kp_[REACH]*(X - (sw_toss *Xb + (1.0f-sw_toss)*Xt) ); 					// DS for approaching the tossing position 
		Eigen::Vector3f Amodul_ee_norm = Kp_[TOSS]*(X - Xb); 						// Modulated DS that aligned  the EE with the desired velocity
		Eigen::Vector3f Amodul_ee_tang = Kp_[TOSS]*(X - Xstar); 				// this->computeModulatedAcceleration(Km, Dm, X, Xdot, Xstar);
		Eigen::Vector3f Aretrac_ee     = Kp_[RETRACT]*(X - Xretr); 			// DS for retracting after the tossing position
		
		// get the modulated motion (out_motion: Velocity)
		Out_motion.head(3) = (1.0-a_retract_)*this->compute_modulated_motion(activation, BasisQ, Areach_ee, Amodul_ee_norm, Amodul_ee_tang) + a_retract_ * Aretrac_ee;
		// get angular motion
		Out_motion.tail(3) = (1.0-a_retract_)*this->compute_angular_motion(coupling_, w_H_ce, Omega, w_H_de, Ko_[REACH], Do_[REACH], this->is2ndOrder_) 
							+   a_retract_   *this->compute_angular_motion(coupling_, w_H_ce, Omega, w_H_re, Ko_[RETRACT], Do_[RETRACT], this->is2ndOrder_);
	}

	// return true;
	return Out_motion;
	
}

Eigen::Vector3f throwingDS::compute_modulated_motion(float activation, Eigen::Matrix3f BasisQ, Eigen::Vector3f Areach_ee, 
																											Eigen::Vector3f Amodul_ee_norm, Eigen::Vector3f Amodul_ee_tang)
{
	//
	Eigen::MatrixXf den_temp  = Areach_ee.transpose() * Areach_ee;
	Eigen::RowVector3f Beta_j = 1.0/(den_temp(0,0)+1e-10) * (Areach_ee.transpose() * BasisQ);

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


Eigen::Vector3f throwingDS::compute_angular_motion(float coupling_, Eigen::Matrix4f w_H_c, Eigen::Vector3f Omega, Eigen::Matrix4f w_H_d, Eigen::Matrix3f Ko, Eigen::Matrix3f Do, bool is2ndOrder){

	// Rotation motion
	//----------------
	Eigen::Matrix3f w_R_c   = w_H_c.block<3,3>(0,0);	// current orientation of the end effector
	Eigen::Matrix3f w_R_d   = w_H_d.block<3,3>(0,0);	// desired orientation of the end effector
	// coupling_ orientation task to position through spherical interpolation of the orientation 
	Eigen::Matrix3f w_R_d_t = Utils<float>::getCombinedRotationMatrix(coupling_, w_R_c, w_R_d); 
	// relative transformation between desired and current frame
	Eigen::Matrix3f d_R_c_t = w_R_d_t.transpose() * w_R_c;

	// 3D Orientation Jacobian 
	Eigen::Matrix3f jacMuTheta = Utils<float>::getMuThetaJacobian(d_R_c_t) * w_R_c.transpose();

	if(is2ndOrder){
		// approaximation of the acceleration (neglecting  -jacMuTheta_dot * Omega)
		return jacMuTheta.inverse() * ( Do*jacMuTheta*Omega + Ko * Utils<float>::getOrientationErrorCur2Des(d_R_c_t));
	}
	else{
		return jacMuTheta.inverse() * (Ko * Utils<float>::getOrientationErrorCur2Des(d_R_c_t));
	}

}

Eigen::MatrixXf throwingDS::createOrthonormalMatrixFromVector(Eigen::VectorXf inVec)
{
  //
  int n = inVec.rows();
  Eigen::MatrixXf basis = Eigen::MatrixXf::Random(n,n); 
  basis.col(0) = 1./inVec.norm() * inVec;

  assert(basis.rows() == basis.cols());
  uint dim = basis.rows();
  basis.col(0).normalize();
  for(uint i=1;i<dim;i++){
      for(uint j=0;j<i;j++)
          basis.col(i) -= basis.col(j).dot(basis.col(i))*basis.col(j);
      basis.col(i).normalize();
  }

  if (basis.rows() == 3){
  	Eigen::Vector3f u = basis.col(0);
  	Eigen::Vector3f v = basis.col(1);
  	Eigen::Vector3f w = u.cross(v);
  	basis.col(2) = w;
  }

  return basis;
} 

bool throwingDS::set_gains(int taskId, int motionId, Eigen::Matrix3f K, Eigen::Matrix3f D){

	if(motionId == TRANSLATION ){
		Kp_[taskId] = K;
		Dp_[taskId] = D;
	}
	else{
		Ko_[taskId] = K;
		Do_[taskId] = D;
	}

	return true;	
}

bool throwingDS::set_modulationParameters(float new_modul_param[]){

	rho_        = new_modul_param[0];
	range_norm_ = new_modul_param[1];
	range_tang_ = new_modul_param[2];

	return true;
}

bool throwingDS::set_toss_linear_velocity(Eigen::Vector3f newLinVel){
	//
	v_toss_ = newLinVel;
	BasisQp_ = this->createOrthonormalMatrixFromVector(v_toss_);
	return true;
}

bool throwingDS::set_toss_angular_velocity(Eigen::Vector3f newAngVel){
	//
	w_toss_  = newAngVel;
	BasisQo_ = this->createOrthonormalMatrixFromVector(w_toss_);
	return true;
}

bool throwingDS::set_toss_pose(Eigen::Vector3f new_releasePos, Eigen::Vector4f new_releaseOrient){
	//
	w_H_de_ = Utils<float>::pose2HomoMx(new_releasePos, new_releaseOrient);
	return true;
}

bool throwingDS::set_rest_pose(Eigen::Vector3f new_restPos, Eigen::Vector4f new_restOrient){
	//
	w_H_re_ = Utils<float>::pose2HomoMx(new_restPos, new_restOrient);
	return true;
}

bool throwingDS::get_release_flag(){
	return release_flag_;
}

bool throwingDS::set_pickup_object_pose(Eigen::Vector3f pickup_Pos, Eigen::Vector4f pickup_Orient){
	//
	w_H_po_ = Utils<float>::pose2HomoMx(pickup_Pos, pickup_Orient);
	return true;
}