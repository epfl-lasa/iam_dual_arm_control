
#include "dualArmCooperativeController.h"
#include "Utils.hpp"

bwc_Vars 		bwc_vars;
bwc_Params 		bwc_params;
bwc_Workspace 	bwc_work;
bwc_Settings 	bwc_settings;

dualArmCooperativeController::dualArmCooperativeController(){}
dualArmCooperativeController::~dualArmCooperativeController(){}

bool dualArmCooperativeController::init()
{
	_tol_dist2contact   = 0.03f;
	_ContactConfidence  = 0.0f;
	_min_Fz 		    = 10.0f; //15.0;  // 40.0f;  
	_min_nF 			= 10.0f;          // 40.0f;
	_max_nF 			= 15.0f;          // 60.0f;
	//
	_mu_ee				= 0.9f;
	_gamma_ee			= 0.9f;
	_deltaX_ee			= 0.5f;
	_deltaY_ee			= 0.5f;
	_contactOccured     = false;
	_targetForce     	= 10.0f;   // 40
	//
	_GraspMatrixEEs.setZero();
	_optimal_contact_wrench_EEs.setZero();
	_optimal_slack.setZero();

	_withForceSaturation = true;

	for(int k=0; k<NB_ROBOTS; k++)
	{
		_dist2contact[k]  = 1.0f;

		_world_Xstar_desEE[k].setIdentity();
		_wrench_correction_ee[k].setZero();
		_complementaryConstraintMatrix[k].setZero();
		_contactConstraintVector[k].setZero();
		_contactConstraintMatrix[k].setZero();
		_f_applied[k].setZero();
		_f_In_EE[k].setZero();
		_nC[k].setZero();
	}

	_weight_EEs_wrench.setOnes();
	_weight_EEs_wrench.segment(0, 3) *= 50.0e-2; 	//1.0e-02  Forces
	_weight_EEs_wrench.segment(3, 3) *= 500.0e-2; 	//1.0e-02 	Moments
	_weight_EEs_wrench.segment(6, 3) *= 50.0e-2; 	//1.0e-02 	Forces
	_weight_EEs_wrench.segment(9, 3) *= 500.0e-2; 	//1.0e-02	Moments	
	//
	_weight_EEs_slack.setZero();
	_weight_EEs_slack << 100.0, 100.0, 100.0, 200.0, 200.0, 200.0;

	// initialization of the cvxgen solver for the cooperative manipulation
	bwc_set_defaults();
	bwc_setup_indexing();

	return true;

}

void dualArmCooperativeController::getGraspKineDynVariables(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[])
{
	// update the grasp matrix
	this->computeBimanualGraspMatrix(w_H_o, w_H_ee, _GraspMatrixEEs);
	//
	for(int k=0; k<NB_ROBOTS; k++)
	{
		_world_Xstar_desEE[k].block<3,3>(0,0) = w_H_ee[k].block<3,3>(0,0);
		_world_Xstar_desEE[k].block<3,3>(3,3) = w_H_ee[k].block<3,3>(0,0);
		_nC[k] = w_H_cp[k].block(0,0,3,3).col(2);
		//
		this->setMinNormalForcesEEs(_min_Fz, w_H_ee[k].block<3,3>(0,0),  _wrench_correction_ee[k]);
	}
}

void dualArmCooperativeController::check_contact_proximity(	Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[])
{
	// Positioning error in hand frames
	Eigen::Vector3f lh_er = w_H_cp[LEFT].block<3,3>(0,0).transpose()*(w_H_cp[LEFT].block<3,1>(0,3)-w_H_ee[LEFT].block<3,1>(0,3)); 	// 
	Eigen::Vector3f rh_er = w_H_cp[RIGHT].block<3,3>(0,0).transpose()*(w_H_cp[RIGHT].block<3,1>(0,3)-w_H_ee[RIGHT].block<3,1>(0,3)); 	// 
	// if norm in x and y are less than thrxy and z less than tol
	//------------------------------------------------------------
	bool tsk = false; 
	if((lh_er.head(2).norm() <= 1.5f*_tol_dist2contact) && (rh_er.head(2).norm() <= 1.5f*_tol_dist2contact)){
		tsk = true;
		_contactOccured = true;
	}
	else {
		tsk = false;
	}
	//
	// Distances normal to contacts
	_dist2contact[LEFT]  = (lh_er(2));
	_dist2contact[RIGHT] = (rh_er(2));
	// update the Contact confidence indicator
	// if(tsk && (fabs(_dist2contact[RIGHT]) <= _tol_dist2contact) && (fabs(_dist2contact[LEFT]) <= _tol_dist2contact)){
	// if(_contactOccured && (fabs(_dist2contact[RIGHT]) <= _tol_dist2contact) && (fabs(_dist2contact[LEFT]) <= _tol_dist2contact)){
	if((fabs(_dist2contact[RIGHT]) <= _tol_dist2contact) && (fabs(_dist2contact[LEFT]) <= _tol_dist2contact)){
		_ContactConfidence = 1.0;
	} else 	{
		_ContactConfidence = 0.0;
	}
	std::cout << " _ContactConfidence 	aaaaaaaaaaaaaaaaaa	---------- is  \t" << _ContactConfidence << std::endl;
	std::cout << " _dist2contact[LEFT] 	aaaaaaaaaaaaaaaaaa	---------- is  \t" << _dist2contact[LEFT] << std::endl;
	std::cout << " _dist2contact[RIGHT] aaaaaaaaaaaaaaaaaa	---------- is  \t" << _dist2contact[RIGHT] << std::endl;
	std::cout << " _lh_er.head(2).norm()aaaaaaaaaaaaaaaaaa	---------- is  \t" << lh_er.head(2).norm() << std::endl;
	std::cout << " _rh_er.head(2).norm()aaaaaaaaaaaaaaaaaa	---------- is  \t" << rh_er.head(2).norm() << std::endl;
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

bool dualArmCooperativeController::computeBimanualGraspMatrix(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix<float,6,12> &GraspMxHands_)
{
	//
	GraspMxHands_.setZero();
	//
	Eigen::Matrix3f skew_Mx_[NB_ROBOTS];
	for(int i=0; i<NB_ROBOTS; i++)
	{
		Eigen::Vector3f t = w_H_ee[i].block<3,1>(0,3) - w_H_o.block<3,1>(0,3);
		skew_Mx_[i] <<      0.0f,   -t(2),      t(1),
		                    t(2),    0.0f,     -t(0),
		                   -t(1),    t(0),      0.0f;             
	}
	// left EE
	GraspMxHands_.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
	GraspMxHands_.block<3,3>(3,0) = -skew_Mx_[LEFT];
	GraspMxHands_.block<3,3>(3,3) = Eigen::Matrix3f::Identity();
	// right EE
	GraspMxHands_.block<3,3>(0,6) = Eigen::Matrix3f::Identity();
	GraspMxHands_.block<3,3>(3,6) = -skew_Mx_[RIGHT];
	GraspMxHands_.block<3,3>(3,9) = Eigen::Matrix3f::Identity();

	return true;
}

void  dualArmCooperativeController::setMinNormalForcesEEs(float min, Eigen::Matrix3f w_R_h, Vector6f &Wrench_w)
{
	// Thresholding normal forces
	Vector6f Wrench_h = Eigen::VectorXf::Zero(6);
	Wrench_h(2) = min;
	//
	Wrench_w.head<3>() = w_R_h*Wrench_h.head<3>();
	Wrench_w.tail<3>() = w_R_h*Wrench_h.tail<3>();
}

void dualArmCooperativeController::thresholdNormalForcesEEs(float min, float max, Eigen::Matrix3f w_R_h, Vector6f &Wrench_w)
{
	// Thresholding normal forces
	Vector6f Wrench_h;
	Wrench_h.head<3>() = w_R_h.transpose()*Wrench_w.head<3>();
	Wrench_h.tail<3>() = w_R_h.transpose()*Wrench_w.tail<3>();

	if(Wrench_h(2) <=min) Wrench_h(2) = min; else if(Wrench_h(2) >= max) Wrench_h(2) = max;
	//
	Wrench_w.head<3>() = w_R_h*Wrench_h.head<3>();
	Wrench_w.tail<3>() = w_R_h*Wrench_h.tail<3>();
}

//
bool dualArmCooperativeController::getComplementaryConstraints(Matrix6f world_Xstar_desEE[], float dist2contact[], float tol_dist2contact)
{
	// thresholding of distance to contact
	float thresh_dist2cnt[NB_ROBOTS];
	for(int k=0; k<NB_ROBOTS; k++)
	{
		thresh_dist2cnt[k] = dist2contact[k];
		if(fabs(dist2contact[k]) <= tol_dist2contact) {
			thresh_dist2cnt[k] = 0.0;
		}
		// transformation from the world to desired lhand frame
		Matrix6f desEE_X_world = world_Xstar_desEE[k].transpose();
		_complementaryConstraintMatrix[k].block<1,6>(0,0)  = thresh_dist2cnt[k] * desEE_X_world.block<1,6>(2,0);
	}
	return true;
}

void dualArmCooperativeController::computeOptimalWrench(Vector6f desired_object_wrench_)
{
	float t_ctrl = ros::Time::now().toSec();
	// get the contact constraints
	this->getContactConstraints(_world_Xstar_desEE);
	// get complementary constraints
	this->getComplementaryConstraints(_world_Xstar_desEE, _dist2contact, _tol_dist2contact);
	// load the data for the solver
	// -----------------------------
	this->load_wrench_data(desired_object_wrench_);

	bwc_settings.verbose = 0;
	//num_iters = 

	// compute the optimal solution (wrench and acceleration)
	// ======================================================
	bwc_solve();
	// Extract the optimal solution vectors
	// ------------------------------------
	// contact forces
	for(int i=0; i<12; i++)
	{
		_optimal_contact_wrench_EEs(i) = bwc_vars.Fh[i];
	}
	// slack variables
	for(int i=0; i<6; i++)
	{
		_optimal_slack(i) = bwc_vars.w[i];
	}
	//
	std::cout << " compute controller solved in " << ros::Time::now().toSec()- t_ctrl << std::endl;

}

bool dualArmCooperativeController::getContactConstraints(Matrix6f world_Xstar_EE[])
{
	for(int k=0; k<NB_ROBOTS; k++)
	{
		Matrix6f ee_X_world = world_Xstar_EE[k].inverse();
		// Inequality Constraint matrix
		_contactConstraintMatrix[k].block<1,6>(0,0)  = -1.0f * ee_X_world.block<1,6>(2,0);
		_contactConstraintMatrix[k].block<1,6>(1,0)  = -_mu_ee/sqrt(2.0f) * ee_X_world.block<1,6>(2,0) - ee_X_world.block<1,6>(0,0);
		_contactConstraintMatrix[k].block<1,6>(2,0)  = -_mu_ee/sqrt(2.0f) * ee_X_world.block<1,6>(2,0) + ee_X_world.block<1,6>(0,0);
		_contactConstraintMatrix[k].block<1,6>(3,0)  = -_mu_ee/sqrt(2.0f) * ee_X_world.block<1,6>(2,0) - ee_X_world.block<1,6>(1,0);
		_contactConstraintMatrix[k].block<1,6>(4,0)  = -_mu_ee/sqrt(2.0f) * ee_X_world.block<1,6>(2,0) + ee_X_world.block<1,6>(1,0);
		_contactConstraintMatrix[k].block<1,6>(5,0)  = 	   	   -_gamma_ee * ee_X_world.block<1,6>(2,0) - ee_X_world.block<1,6>(5,0);
		_contactConstraintMatrix[k].block<1,6>(6,0)  = 	       -_gamma_ee * ee_X_world.block<1,6>(2,0) + ee_X_world.block<1,6>(5,0);
		_contactConstraintMatrix[k].block<1,6>(7,0)  = 	      -_deltaX_ee * ee_X_world.block<1,6>(2,0) - ee_X_world.block<1,6>(4,0);
		_contactConstraintMatrix[k].block<1,6>(8,0)  = 	      -_deltaX_ee * ee_X_world.block<1,6>(2,0) + ee_X_world.block<1,6>(4,0);
		_contactConstraintMatrix[k].block<1,6>(9,0)  = 	      -_deltaY_ee * ee_X_world.block<1,6>(2,0) - ee_X_world.block<1,6>(3,0);
		_contactConstraintMatrix[k].block<1,6>(10,0) = 	      -_deltaY_ee * ee_X_world.block<1,6>(2,0) + ee_X_world.block<1,6>(3,0);
		// Inequality Constraint vector 
		_contactConstraintVector[k](0)  = _min_nF;  // minimal normal forces
		_contactConstraintVector[k](11) = _max_nF;  // maximun normal forces
	}
	return true;
}


void dualArmCooperativeController::load_wrench_data(Vector6f desired_object_wrench_)
{

	// weight hands wrench
	for(int i=0; i<12; i++)
	{
		bwc_params.QFh[i] = _weight_EEs_wrench(i);
	}
	// weight hands slack
	for(int i=0; i<6; i++)
	{
		bwc_params.Qw[i] = _weight_EEs_slack(i);
	}

	// desired contact wrench wrench
	for(int i=0; i<6; i++)
	{
		bwc_params.pFh[i]   = 0.0f + _wrench_correction_ee[LEFT](i);
		bwc_params.pFh[6+i] = 0.0f + _wrench_correction_ee[RIGHT](i);
	}
	// bwc_params.pFh[2] =   10.0 ;
	// bwc_params.pFh[8] =  -10.0 ;

	// contact activator	
	bwc_params.beta[0] = _ContactConfidence;  

	// Grasp Matrix
	for(int i=0; i<12; i++)
	{
		// bwc_params.Gh_1[i] = _GraspMatrixEEs(0,i);
		// bwc_params.Gh_2[i] = _GraspMatrixEEs(1,i); 
		// bwc_params.Gh_3[i] = _GraspMatrixEEs(2,i); 
		bwc_params.Gh_4[i] = _GraspMatrixEEs(3,i);
		bwc_params.Gh_5[i] = _GraspMatrixEEs(4,i);
		bwc_params.Gh_6[i] = _GraspMatrixEEs(5,i);
	}

	// b1 == Mo*ddot_Xo + bo - fenv
	for(int i=0; i<6; i++)
	{
		bwc_params.b1[i] = desired_object_wrench_(i);
	}

	// complementary condition
	for (int i=0; i<6; i++)
	{
		bwc_params.Cplh[i] =  _complementaryConstraintMatrix[LEFT](0,i);
		bwc_params.Cprh[i] =  _complementaryConstraintMatrix[RIGHT](0,i);
	}

	// contact constaints
	for(int i=0; i<6; i++)
	{
		// _contactConstraintMatrix[LEFT] *= 0.0;
		// _contactConstraintMatrix[RIGHT] *= 0.0;
		bwc_params.CLH_1[i] = _contactConstraintMatrix[LEFT](0,i);		bwc_params.CLH_7[i]  = _contactConstraintMatrix[LEFT](6,i);
		bwc_params.CLH_2[i] = _contactConstraintMatrix[LEFT](1,i);		bwc_params.CLH_8[i]  = _contactConstraintMatrix[LEFT](7,i);
		bwc_params.CLH_3[i] = _contactConstraintMatrix[LEFT](2,i);		bwc_params.CLH_9[i]  = _contactConstraintMatrix[LEFT](8,i);
		bwc_params.CLH_4[i] = _contactConstraintMatrix[LEFT](3,i);		bwc_params.CLH_10[i] = _contactConstraintMatrix[LEFT](9,i);
		bwc_params.CLH_5[i] = _contactConstraintMatrix[LEFT](4,i);		bwc_params.CLH_11[i] = _contactConstraintMatrix[LEFT](10,i);
		bwc_params.CLH_6[i] = _contactConstraintMatrix[LEFT](5,i);

		bwc_params.CRH_1[i] = _contactConstraintMatrix[RIGHT](0,i);		bwc_params.CRH_7[i]  = _contactConstraintMatrix[RIGHT](6,i);
		bwc_params.CRH_2[i] = _contactConstraintMatrix[RIGHT](1,i);		bwc_params.CRH_8[i]  = _contactConstraintMatrix[RIGHT](7,i);
		bwc_params.CRH_3[i] = _contactConstraintMatrix[RIGHT](2,i);		bwc_params.CRH_9[i]  = _contactConstraintMatrix[RIGHT](8,i);
		bwc_params.CRH_4[i] = _contactConstraintMatrix[RIGHT](3,i);		bwc_params.CRH_10[i] = _contactConstraintMatrix[RIGHT](9,i);
		bwc_params.CRH_5[i] = _contactConstraintMatrix[RIGHT](4,i);		bwc_params.CRH_11[i] = _contactConstraintMatrix[RIGHT](10,i);
		bwc_params.CRH_6[i] = _contactConstraintMatrix[RIGHT](5,i);
	}
	// for(int i=0; i<12; i++)
	// {
	// 	bwc_params.blh[i] =  _contactConstraintVector[LEFT](i);
	// 	bwc_params.brh[i] =  _contactConstraintVector[RIGHT](i);
	// }

}



void dualArmCooperativeController::computeControlWrench(Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[], Vector6f desired_object_wrench_)
{
	//
	this->getGraspKineDynVariables(w_H_o, w_H_ee, w_H_cp);
	this->check_contact_proximity(w_H_ee, w_H_cp);
	this->computeOptimalWrench(desired_object_wrench_);

	// Extraction of left and right hands wrenches
	_f_applied[LEFT]  = _optimal_contact_wrench_EEs.head<6>();
	_f_applied[RIGHT] = _optimal_contact_wrench_EEs.tail<6>();
	// Normal forces saturation and moments correction
	_f_In_EE[LEFT].head<3>()  = w_H_ee[LEFT].block<3,3>(0,0).transpose()*_f_applied[LEFT].head<3>();
	_f_In_EE[LEFT].head<3>()  = w_H_ee[LEFT].block<3,3>(0,0).transpose()*_f_applied[LEFT].head<3>();
	_f_In_EE[RIGHT].head<3>() = w_H_ee[RIGHT].block<3,3>(0,0).transpose()*_f_applied[RIGHT].tail<3>();
	_f_In_EE[RIGHT].head<3>() = w_H_ee[RIGHT].block<3,3>(0,0).transpose()*_f_applied[RIGHT].tail<3>();

	// Thresholding normal forces
	if(_withForceSaturation) {
		this->thresholdNormalForcesEEs(_min_nF, _max_nF, w_H_ee[LEFT].block<3,3>(0,0),  _f_applied[LEFT]); 
    this->thresholdNormalForcesEEs(_min_nF, _max_nF, w_H_ee[RIGHT].block<3,3>(0,0), _f_applied[RIGHT]);
	}
	// Printing some results
	// ---------------------
	std::cout << " OPTIMAL HAND WRENCH   LEFT \t " << _optimal_contact_wrench_EEs.head(6).transpose() << std::endl;
	std::cout << " OPTIMAL HAND WRENCH  RIGHT \t " << _optimal_contact_wrench_EEs.tail(6).transpose() << std::endl;
  std::cout << " APPLIED HAND WRENCH  LEFT \t " << _f_applied[LEFT].transpose() << std::endl;
	std::cout << " APPLIED HAND WRENCH RIGHT \t " << _f_applied[RIGHT].transpose() << std::endl;

}


//
void dualArmCooperativeController::getPredefinedContactForceProfile(bool goHome, int contactState, Eigen::Matrix4f w_H_o, Eigen::Matrix4f w_H_ee[], Eigen::Matrix4f w_H_cp[])
{
  //
	this->getGraspKineDynVariables(w_H_o, w_H_ee, w_H_cp);
	//
	_f_applied[LEFT].setZero(); 
	_f_applied[RIGHT].setZero();
	//
  if(goHome)
  {
    _f_applied[LEFT].setZero(); 
    _f_applied[RIGHT].setZero();
  }
  else
  {
    if(contactState==CONTACT)
    {
			_f_applied[LEFT].head(3)  = _targetForce * _nC[LEFT];
			_f_applied[RIGHT].head(3) = _targetForce * _nC[RIGHT];
    }
    else if(contactState==CLOSE_TO_CONTACT)
    {
      _f_applied[LEFT].head(3)  = 8.0f * _nC[LEFT];
			_f_applied[RIGHT].head(3) = 8.0f * _nC[RIGHT];
    }
    else
    {
    	_f_applied[LEFT].setZero(); 
    	_f_applied[RIGHT].setZero();
    }
  }

  std::cout << " APPLIED HAND WRENCH  LEFT \t " << _f_applied[LEFT].transpose() << std::endl;
	std::cout << " APPLIED HAND WRENCH RIGHT \t " << _f_applied[RIGHT].transpose() << std::endl;
}