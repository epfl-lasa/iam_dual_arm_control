
#include "iam_dual_arm_control/toss_task_param_estimator.h"


//
toss_task_param_estimator::toss_task_param_estimator(){}

toss_task_param_estimator::~toss_task_param_estimator(){}


void toss_task_param_estimator::init(std::string file_gmm[], Eigen::Vector3f x_release, Eigen::Vector4f q_release, Eigen::Vector3f v_release, Eigen::Vector3f w_release){

	x_release_ = x_release;
	q_release_ = q_release;
	v_release_ = v_release;
	w_release_ = w_release;

	myGMR.init(file_gmm);
	// datalog.Load_gmm_param2(file_gmm, Prior_gmm_toss_, Mean_gmm_toss_, CovMx_gmm_toss_);
}

void toss_task_param_estimator::cartesian2planar(Eigen::Vector3f Pos_3d_in, Eigen::Vector2f &pos_bar, float &phi_d, float &theta_d)
{
	//Coordinates transformations
    //convert cartesian into planar data using cylindrical coordinates
    pos_bar(0) = Pos_3d_in.head(2).norm();
    pos_bar(1) = Pos_3d_in(2);

    phi_d   = std::atan2(Pos_3d_in(1), Pos_3d_in(0));
    // direction of desired positions
    theta_d = std::atan2(pos_bar(1), pos_bar(0));
}

Eigen::Vector3f toss_task_param_estimator::cartesian2planar(Eigen::Vector3f Pos_d){
	//
	Eigen::Vector3f out = Eigen::VectorXf::Zero(3);
	out(0) = Pos_d.head(2).norm();  									// r
  out(1) = Pos_d(2);																// z
  out(2) = std::atan2(Pos_d(1), Pos_d(0));					// phi
	return out;
}

Eigen::Vector3f toss_task_param_estimator::cartesian2spherical(Eigen::Vector3f Pos_d){
	//
	Eigen::Vector3f out = Eigen::VectorXf::Zero(3);
	out(0) = Pos_d.norm(); 																// r
  // out(1) = std::acos( Pos_d(2), out(0));							// theta
  out(1) = std::atan2( Pos_d.head(2).norm(), Pos_d(2)); // theta				
  out(2) = std::atan2(Pos_d(1), Pos_d(0));							// phi

	return out;
}

Eigen::Vector3f toss_task_param_estimator::get_state_variation(float dt, Eigen::Vector3f X, Eigen::Vector3f Xd, Eigen::Vector3f dX_d){ // DO REPLACE DS_ROBOT
	//
	Eigen::Vector3f X1, X2, X3;
	Eigen::Vector3f k_1, k_2, k_3, k_4;

	k_1 = dX_d;
	X1  = X + 0.5*dt*k_1;
	k_2 = dX_d;
	X2  = X + 0.5*dt*k_2;
	k_3 = dX_d;
	X3  = X + dt*k_3;
	k_4 = dX_d;

	return dt/6.f * (k_1 + 2.f*k_2 + 2.f*k_3 + k_4);
}
//
void toss_task_param_estimator::get_min_release_speed(Eigen::Vector2f pos_d, float &ang_release_i, float &v_release_i, float &t_flight_i)
{

	float theta_d = std::atan2(pos_d(1), pos_d(0));
    // function to estimate the minimum release velocity based of an known release angle
   	float theta_i = std::atan(pos_d(1)/pos_d(0) + std::sqrt((pos_d(1)/pos_d(0))*(pos_d(1)/pos_d(0)) + 1.f));

   	if(theta_i > M_PI/2.f){
   		theta_i = (90.f-5.f)/180.f * M_PI;
   	}
		else if (theta_i < theta_d){
			theta_i = theta_d + 5.f/180.f * M_PI;
		}
	
    float delta_r = pos_d(0);
    float delta_z = pos_d(1);
    //
    float num = (this->g * delta_r*delta_r * (1.f+std::tan(theta_i)*std::tan(theta_i)));
    float den = (2.f * (delta_r * std::tan(theta_i) - delta_z));
    // get the initial velocity in (m/s)
    v_release_i = fabs(std::sqrt(num / den));

    // 
    t_flight_i = fabs((1./this->g)*(v_release_i*std::sin(theta_i) + std::sqrt(std::sin(theta_i)*std::sin(theta_i) + 2.f*this->g*delta_z)));
}

//
void toss_task_param_estimator::estimate_2d_throwing_param(ProjectileType type, Eigen::Vector2f pos_d,  
															float &ang_release_i, float &v_release_i, float &t_flight_i)
{
	//
	if(type == PHYS_IDEAL){
		this->get_min_release_speed(pos_d, ang_release_i, v_release_i, t_flight_i);
	}
	else if(type == PHYS_WITH_DRAG){
		this->get_min_release_speed(pos_d, ang_release_i, v_release_i, t_flight_i);	// TO DO replace with appropriate function
	}
	else{ //(LEARNED)

		float ang_release, v_release;
		this->get_min_release_speed(pos_d, ang_release, v_release, t_flight_i);	// TO DO replace with appropriate function
		//
		Eigen::MatrixXf E_Cov_I;
		Eigen::MatrixXf E_Cov_O;
		Eigen::VectorXf theta_vel_release;

		// this->get_GMR_IO(this->Prior_gmm_toss_, this->Mean_gmm_toss_, this->CovMx_gmm_toss_, pos_d,  theta_vel_release, E_Cov_I, E_Cov_O);

		myGMR.compute_gmr_outputs(pos_d,  theta_vel_release, E_Cov_I, E_Cov_O);

		ang_release_i = theta_vel_release(0);
		v_release_i   = theta_vel_release(1);
	}

	//
}

//
bool toss_task_param_estimator::estimate_tossing_param(ProjectileType type, Eigen::Vector3f Pos_landing, Eigen::Vector3f Pos_Release){
	//
	Eigen::Vector3f Pos_3d_in = Pos_landing - Pos_Release;

	Eigen::Vector2f pos_bar;  
	float ang_release_i; 
	float v_release_i; 
	float t_flight_i;
	float theta_d;
	float phi_d;

	//
	this->cartesian2planar(Pos_3d_in, pos_bar, phi_d, theta_d); 							  // in: Pos_3d_in --> out : pos_bar, phi_d, theta_d

	this->estimate_2d_throwing_param(type, pos_bar,  ang_release_i, v_release_i, t_flight_i); // --> out: ang_release_i, v_release_i, t_flight_i

	//
	this->x_release_    = Pos_Release;
	this->v_release_(0) = v_release_i*cos(ang_release_i)*cos(phi_d);
	this->v_release_(1) = v_release_i*cos(ang_release_i)*sin(phi_d);
	this->v_release_(2) = v_release_i*sin(ang_release_i);

}

Eigen::Vector3f toss_task_param_estimator::get_release_position(){
	return x_release_;
}
Eigen::Vector4f toss_task_param_estimator::get_release_orientation(){
	return q_release_;
}
Eigen::Vector3f toss_task_param_estimator::get_release_linear_velocity(){
	return v_release_;
}
Eigen::Vector3f toss_task_param_estimator::get_release_angular_velocity(){
	return w_release_;
}




Eigen::Vector4f toss_task_param_estimator::ds_projectile2d(Eigen::Vector4f rz, float g, float mu){

	// dynamics of a projectile with newton air drag 
	Eigen::Vector4f dxdt = Eigen::VectorXf::Zero(4);

	float v_rz = rz.tail(2).norm();
	dxdt << rz(2), rz(3), -mu*rz(2)*v_rz, -mu*rz(3)*v_rz-g;

	return dxdt;
}

void toss_task_param_estimator::projectileMotion2d(float T, float g, float mu, Eigen::Vector2f pos_i, Eigen::Vector2f pos_d, float v0_i, float theta_i, 
																									 float& flytime, Eigen::Vector2f& Xland2d){
	float dt = T; // intergration time
	float tol = 1e-5;
	Eigen::Vector4f rz_0 = {pos_i(0), pos_i(1), v0_i*cos(theta_i), v0_i*sin(theta_i)};
	Eigen::Vector4f rz = rz_0;
	Eigen::Vector2f rz_rot = rz_0.head(2);

	float theta_dd  	 = std::atan2(pos_d(1), pos_d(0));
	Eigen::Matrix2f Rt;
	Rt << cos(theta_dd), sin(theta_dd),
			 -sin(theta_dd), cos(theta_dd);
	// projectile dynamics
	Eigen::Vector4f dxdt  = ds_projectile2d(rz,g,mu);

	int iter = 0;
	bool isStopping = true;

	myRK4.InitializeFilter(T, 1.f, 0.f, rz);
	flytime = 0.0f;


	while(isStopping){
		if((T > 0.001f) && (rz.tail(2).norm() >5.f)){
			dt = 0.001;
			myRK4.setSampleTime(dt);
		}   
    else{
      dt = T;
      myRK4.setSampleTime(dt);
    }
    // integrate
    rz    = myRK4.getRK4Integral(dxdt);
    rz_rot= Rt*rz.head(2);
    // 
    dxdt  = ds_projectile2d(rz,g,mu);
    flytime +=dt;
    iter    +=1;

    if(theta_dd < 0.f){
    	isStopping = ((rz(1)+tol >= pos_d(1)) && (rz_rot(0)+tol != rz_0(0)) && iter<=2000);
    }
    else{
    	isStopping = ((rz_rot(1)+tol >= 0) && (rz_rot(0)+tol != rz_0(0)) && iter<=2000);
    }
	}

	Xland2d = rz.head(2);
	flytime = flytime;
}


void toss_task_param_estimator::estimate_landing_state(float T, float g, float mu, Vector7f Xd_land, Vector7f Xrelease, Eigen::Vector3f Vel_feas, float &flytime, Eigen::Vector3f &Xf_land){
	//
	Eigen::Vector2f pos_2d_i = {0.f, 0.f};
	Eigen::Vector3f X3d_bar  = Xd_land.head(3) - Xrelease.head(3);

	Eigen::Vector3f Xplanar    = cartesian2planar(X3d_bar);
	Eigen::Vector3f Xspherical = cartesian2spherical(Vel_feas);
	float theta_r = M_PI/2.f - Xspherical(1);

	Eigen::Vector2f Xland2d;
	projectileMotion2d(T, g, mu, pos_2d_i, Xplanar.head(2), Xspherical(0), theta_r, flytime, Xland2d);

	Xf_land << Xland2d(0)*std::cos(Xspherical(2))+Xrelease(0),
						 Xland2d(0)*std::sin(Xspherical(2))+Xrelease(1),
						 Xland2d(1)+Xrelease(2);
}

Eigen::Vector2f toss_task_param_estimator::estimateTarget_SimpPathLength_AverageSpeed(Eigen::Vector3f Xt, Eigen::Vector3f Xt_d, Eigen::Vector3f aVtarget){
	//
	float v_avg  = aVtarget.norm();
	float LpXd_X = (Xt_d - Xt).norm();
	Eigen::Vector2f Lp_dx_avg = {LpXd_X, v_avg}; // path length and averge velocity
	return Lp_dx_avg;
}


Eigen::Vector3f toss_task_param_estimator::estimate_target_state_to_go(Eigen::Vector3f Xtarget, Eigen::Vector3f dX_target, Eigen::Vector3f Xintercept, Eigen::Vector2f Lp_Va_pred_bot, Eigen::Vector2f Lp_Va_pred_tgt, float flytime_obj)
{
	float Lptgt2Xintercept = 0.0f;

	if(Lp_Va_pred_bot(1) >= 1e-4){
		Lptgt2Xintercept = Lp_Va_pred_tgt(1) * (Lp_Va_pred_bot(0)/Lp_Va_pred_bot(1) + flytime_obj);  // avg_speed_target*(robot_PathLength/avg_speed_robot + object_flying_time)
	}
	else{
		Lptgt2Xintercept = 0.0f;
	}
	Eigen::Vector3f Xtarget2go = Xintercept -dX_target.normalized() * Lptgt2Xintercept;  		// relative to Xintercept

	// if sign ()
	return Xtarget2go; 
}