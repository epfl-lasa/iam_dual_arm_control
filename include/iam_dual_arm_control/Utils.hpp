#ifndef __UTILS_H__
#define __UTILS_H__

#include "ros/ros.h"
#include "Eigen/Eigen"


template<typename T = float>
class Utils 
{
	public:

	enum ROBOT_ID {KUKA_LWR, FRANKA_PANDA};


	enum DH_CONVENTION {NORMAL, MODIFIED};

	// Class constructor
	Utils(){}

    static Eigen::Matrix<T,4,1> quaternionProduct(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2)
    {
	  Eigen::Matrix<T,4,1> q;
	  q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
	  Eigen::Matrix<T,3,1> q1Im = (q1.segment(1,3));
	  Eigen::Matrix<T,3,1> q2Im = (q2.segment(1,3));
	  q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

	  return q;
	}

    static Eigen::Matrix<T,3,3> getSkewSymmetricMatrix(Eigen::Matrix<T,3,1> input)
    {
	  Eigen::Matrix<T,3,3> output;

	  output << 0.0f, -input(2), input(1),
	            input(2), 0.0f, -input(0),
	            -input(1), input(0), 0.0f;

	  return output;
	}

	static Eigen::Matrix<T,3,3> eulerAnglesToRotationMatrix(T phi, T theta, T psi)
  	{
	  T cphi = std::cos(phi);
	  T sphi = std::sin(phi);
	  T ctheta = std::cos(theta);
	  T stheta = std::sin(theta);
	  T cpsi = std::cos(psi);
	  T spsi = std::sin(psi);


	  Eigen::Matrix<T,3,3> R;
	  R << cpsi*ctheta, cpsi*stheta*sphi-spsi*cphi, cpsi*stheta*cphi+spsi*sphi,
	       spsi*ctheta, spsi*stheta*sphi+cpsi*cphi, spsi*stheta*cphi-cpsi*sphi,
	       -stheta, ctheta*sphi, ctheta*cphi;

	  return R;
	}

    static Eigen::Matrix<T,4,1> rotationMatrixToQuaternion(Eigen::Matrix<T,3,3> R)
    {
	  Eigen::Matrix<T,4,1> q;

	  float r11 = R(0,0);
	  float r12 = R(0,1);
	  float r13 = R(0,2);
	  float r21 = R(1,0);
	  float r22 = R(1,1);
	  float r23 = R(1,2);
	  float r31 = R(2,0);
	  float r32 = R(2,1);
	  float r33 = R(2,2);

	  float tr = r11+r22+r33;
	  float tr1 = r11-r22-r33;
	  float tr2 = -r11+r22-r33;
	  float tr3 = -r11-r22+r33;

	  if(tr>0)
	  {  
	    q(0) = sqrt(1.0f+tr)/2.0f;
	    q(1) = (r32-r23)/(4.0f*q(0));
	    q(2) = (r13-r31)/(4.0f*q(0));
	    q(3) = (r21-r12)/(4.0f*q(0));
	  }
	  else if((tr1>tr2) && (tr1>tr3))
	  {
	    q(1) = sqrt(1.0f+tr1)/2.0f;
	    q(0) = (r32-r23)/(4.0f*q(1));
	    q(2) = (r21+r12)/(4.0f*q(1));
	    q(3) = (r31+r13)/(4.0f*q(1));
	  }     
	  else if((tr2>tr1) && (tr2>tr3))
	  {   
	    q(2) = sqrt(1.0f+tr2)/2.0f;
	    q(0) = (r13-r31)/(4.0f*q(2));
	    q(1) = (r21+r12)/(4.0f*q(2));
	    q(3) = (r32+r23)/(4.0f*q(2));
	  }
	  else
	  {
	    q(3) = sqrt(1.0f+tr3)/2.0f;
	    q(0) = (r21-r12)/(4.0f*q(3));
	    q(1) = (r31+r13)/(4.0f*q(3));
	    q(2) = (r32+r23)/(4.0f*q(3));        
	  }

	  return q;
	}

  	static Eigen::Matrix<T,3,3> quaternionToRotationMatrix(Eigen::Matrix<T,4,1> q)
  	{
	  Eigen::Matrix<T,3,3> R;

	  T q0 = q(0);
	  T q1 = q(1);
	  T q2 = q(2);
	  T q3 = q(3);

	  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
	  R(1,0) = 2.0f*(q1*q2+q0*q3);
	  R(2,0) = 2.0f*(q1*q3-q0*q2);

	  R(0,1) = 2.0f*(q1*q2-q0*q3);
	  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
	  R(2,1) = 2.0f*(q2*q3+q0*q1);

	  R(0,2) = 2.0f*(q1*q3+q0*q2);
	  R(1,2) = 2.0f*(q2*q3-q0*q1);
	  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

	  return R;
	}

	static void quaternionToAxisAngle(Eigen::Matrix<T,4,1> q, Eigen::Matrix<T,3,1> &axis, T &angle)
	{
	  if((q.segment(1,3)).norm() < 1e-3f)
	  {
	    axis = q.segment(1,3);
	  }
	  else
	  {
	    axis = q.segment(1,3)/(q.segment(1,3)).norm();
	    
	  }

	  angle = 2*std::acos(q(0));
	}

	static Eigen::Matrix<T,4,1> axisAngleToQuaterion(Eigen::Matrix<T,3,1> axis, T angle)
	{
	  Eigen::Matrix<T,4,1> q;
	  q(0) = std::cos(angle/2);
	  q(1) = axis(0)*std::sin(angle/2);
	  q(2) = axis(1)*std::sin(angle/2);
	  q(3) = axis(2)*std::sin(angle/2);
	  return q;
	}

  	static Eigen::Matrix<T,4,1> slerpQuaternion(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2, T t)
  	{

	  Eigen::Matrix<T,4,1> q;

	  // Change sign of q2 if dot product of the two quaterion is negative => allows to interpolate along the shortest path
	  if(q1.dot(q2)<0.0f)
	  {   
	    q2 = -q2;
	  }

	  T dotProduct = q1.dot(q2);
	  if(dotProduct > 1.0f)
	  {
	    dotProduct = 1.0f;
	  }
	  else if(dotProduct < -1.0f)
	  {
	    dotProduct = -1.0f;
	  }

	  T omega = acos(dotProduct);

	  if(std::fabs(omega)<FLT_EPSILON)
	  {
	    q = q1.transpose()+t*(q2-q1).transpose();
	  }
	  else
	  {
	    q = (std::sin((1-t)*omega)*q1+std::sin(t*omega)*q2)/std::sin(omega);
	  }

	  return q;
	}

	static Eigen::Matrix<T,4,1> slerpQuaternion(Eigen::Matrix<T,4,1>* q, Eigen::Matrix<T,Eigen::Dynamic,1> t, int size)
	{

	  if(size==1)
	  {
	    return q[0];
	  }
	  else
	  {
	    T sum = 0.0f;
	    for(int k = 0; k <size; k++)
	    {
	      sum+=t(k);
	    }
	    if(sum<FLT_EPSILON)
	    {
	      return slerpQuaternion(slerpQuaternion(q,t,size-1),q[size-1],0.0f);
	    }
	    else
	    {
	      return slerpQuaternion(slerpQuaternion(q,t,size-1),q[size-1],t(size-1)/sum);
	    }
	  }
}

	static Eigen::Matrix<T,3,3> rodriguesRotation(Eigen::Matrix<T,3,1> v1, Eigen::Matrix<T,3,1> v2)
	{
	  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
	  v1.normalize();
	  v2.normalize();

	  Eigen::Matrix<T,3,1> w;
	  w = v1.cross(v2);
	  float c = v1.dot(v2);  
	  float s = w.norm();
	  w /= s;
	  
	  Eigen::Matrix<T,3,3> K;
	  K << getSkewSymmetricMatrix(w);

	  Eigen::Matrix<T,3,3> Re;
	  if(fabs(s)< FLT_EPSILON)
	  {
	    Re = Eigen::Matrix<T,3,3>::Identity();
	  }
	  else
	  {
	    Re = Eigen::Matrix<T,3,3>::Identity()+s*K+(1-c)*K*K;
	  }

	  return Re;
	}

	static Eigen::Matrix<T,3,1> quaternionToAngularVelocity(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2, T gain = 1.0f)
	{
	  Eigen::Matrix<T,4,1> q1I, wq;
	  q1I(0) = q1(0);
	  q1I.segment(1,3) = -q1.segment(1,3);
	  wq = 2.0f*gain*quaternionProduct(q2-q1,q1I);
	  
	  return wq.segment(1,3);
	}


	static Eigen::Matrix<T,3,3> orthogonalProjector(Eigen::Matrix<T,3,1> v)
	{ 
	  return Eigen::Matrix<T,3,3>::Identity()-v*v.transpose();
	}

  	static T smoothRise(T x, T a, T b)
  	{
	  T y; 
	  if(x<a)
	  {
	    y = 0.0f;
	  }
	  else if(x>b)
	  {
	    y = 1.0f;
	  }
	  else
	  {
	    y = (1.0f+sin(M_PI*(x-a)/(b-a)-M_PI/2.0f))/2.0f;
	  }

	  return y;
	}


	static T smoothFall(T x, T a, T b)
	{
	  return 1.0f-smoothRise(x,a,b);
	}


	static T smoothRiseFall(T x, T a, T b, T c, T d)
	{
	  return smoothRise(x,a,b)*smoothFall(x,c,d);
	}


	static T deadZone(T x, T a, T b)
	{
	  if(x < b && x > a)
	  {
	    return 0.0f;
	  }
	  else
	  {
	    return x;
	  }
	}

	static Eigen::Matrix<T,Eigen::Dynamic,1> deadZone(Eigen::Matrix<T,Eigen::Dynamic,1> x, T limit)
	{
	  T norm = x.norm();

	  if(norm>limit)
	  {
	    return x;
	  }
	  else
	  {
	    return Eigen::Matrix<T,Eigen::Dynamic,1>::Zero(x.size());
	  }
	}


	static T wrapToZero(T x, T a, T b)
	{
	  if(x < b && x > a)
	  {
	    return x;
	  }
	  else
	  {
	    return 0.0f;
	  }
	}


	static T bound(T x, T a, T b)
	{
	  if(x > b)
	  {
	    return b;
	  }
	  else if(x<a)
	  {
	    return a;
	  }
	  else
	  {
	    return x;
	  }
	}

	static Eigen::Matrix<T,Eigen::Dynamic,1> bound(Eigen::Matrix<T,Eigen::Dynamic,1> x, T limit)
	{
	  T norm = x.norm();

	  if(norm>limit)
	  {
	    return x*limit/norm;
	  }
	  else
	  {
	    return x;
	  }
	}

	static Eigen::Matrix<T,4,4> getDHMatrix(T a, T alpha, T d, T theta, DH_CONVENTION dhConvention = NORMAL)
	{
	  Eigen::Matrix<T,4,4> H;

	  if(dhConvention == NORMAL)
	  {
	    H(0,0) = std::cos(theta);
	    H(0,1) = -std::cos(alpha)*std::sin(theta);
	    H(0,2) = std::sin(alpha)*std::sin(theta);
	    H(0,3) = a*std::cos(theta);

	    H(1,0) = std::sin(theta);
	    H(1,1) = std::cos(alpha)*std::cos(theta);
	    H(1,2) = -std::sin(alpha)*std::cos(theta);
	    H(1,3) = a*std::sin(theta);

	    H(2,0) = 0.0f;
	    H(2,1) = std::sin(alpha);
	    H(2,2) = std::cos(alpha);
	    H(2,3) = d;

	    H(3,0) = 0.0f;
	    H(3,1) = 0.0f;
	    H(3,2) = 0.0f;
	    H(3,3) = 1.0f;
	  }
	  else
	  {
	    H(0,0) = std::cos(theta);
	    H(0,1) = -std::sin(theta);
	    H(0,2) = 0.0f;
	    H(0,3) = a;

	    H(1,0) = std::sin(theta)*std::cos(alpha);
	    H(1,1) = std::cos(theta)*std::cos(alpha);
	    H(1,2) = -std::sin(alpha);
	    H(1,3) = -d*std::sin(alpha);

	    H(2,0) = std::sin(theta)*std::sin(alpha);
	    H(2,1) = std::cos(theta)*std::sin(alpha);
	    H(2,2) = std::cos(alpha);
	    H(2,3) = d*std::cos(alpha);

	    H(3,0) = 0.0f;
	    H(3,1) = 0.0f;
	    H(3,2) = 0.0f;
	    H(3,3) = 1.0f;
	  }

	  return H;
	}

	static Eigen::Matrix<T,4,4> getForwardKinematics(Eigen::Matrix<T,7,1> joints, ROBOT_ID robotID = KUKA_LWR)
	{
	  Eigen::Matrix<T,4,4> H, H1, H2, H3, H4, H5, H6, H7, H8;

	  if(robotID==KUKA_LWR)
	  {
	    H1 = getDHMatrix(0.0f,M_PI/2.0f,0.3105f,joints(0));
	    H2 = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1));
	    H3 = getDHMatrix(0.0f,-M_PI/2.0f,0.4f,joints(2));
	    H4 = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(3));
	    H5 = getDHMatrix(0.0f,M_PI/2.0f,0.39f,joints(4));
	    H6 = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(5));
	    H7 = getDHMatrix(0.0f,0.0f,0.078f,joints(6));    
	    H = H1*H2*H3*H4*H5*H6*H7;
	  }
	  else if(robotID==FRANKA_PANDA)
	  {
	    H1 = getDHMatrix(0.0f,0.0f,0.333f,joints(0),MODIFIED);
	    H2 = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1),MODIFIED);
	    H3 = getDHMatrix(0.0f,M_PI/2.0f,0.316f,joints(2),MODIFIED);
	    H4 = getDHMatrix(0.0825f,M_PI/2.0f,0.0f,joints(3),MODIFIED);
	    H5 = getDHMatrix(-0.0825f,-M_PI/2.0f,0.384f,joints(4),MODIFIED);
	    H6 = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(5),MODIFIED);
	    H7 = getDHMatrix(0.088f,M_PI/2.0f,0.0f,joints(6),MODIFIED);  
	    H8 = getDHMatrix(0.0f,0.0f,0.107f,0.0f,MODIFIED); 
	    H = H1*H2*H3*H4*H5*H6*H7*H8;
	  }

	  return H;
	}


	static Eigen::Matrix<T,6,7> getGeometricJacobian(Eigen::Matrix<T,7,1> joints, Eigen::Matrix<T,3,1> rEEx = Eigen::Matrix<T,3,1>::Zero(), ROBOT_ID robotID = KUKA_LWR)
	{
	  Eigen::Matrix<T,4,4> Hee, H[8], Hk;
	  Eigen::Matrix<T,6,7> J;

	  DH_CONVENTION convention;
	  if(robotID == KUKA_LWR)
	  {
	    convention = NORMAL;
	  }
	  else if(robotID = FRANKA_PANDA)
	  {
	    convention = MODIFIED;
	  }

	  if(robotID == KUKA_LWR)
	  {  
	    H[0] = getDHMatrix(0.0f,M_PI/2.0f,0.3105f,joints(0));
	    H[1] = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1));
	    H[2] = getDHMatrix(0.0f,-M_PI/2.0f,0.4f,joints(2));
	    H[3] = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(3));
	    H[4] = getDHMatrix(0.0f,M_PI/2.0f,0.39f,joints(4));
	    H[5] = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(5));
	    H[6] = getDHMatrix(0.0f,0.0f,0.078f,joints(6));
	    Hee = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6];
	  }
	  else if(robotID == FRANKA_PANDA)
	  {
	    H[0] = getDHMatrix(0.0f,0.0f,0.333f,joints(0),convention);
	    H[1] = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1),convention);
	    H[2] = getDHMatrix(0.0f,M_PI/2.0f,0.316f,joints(2),convention);
	    H[3] = getDHMatrix(0.0825f,M_PI/2.0f,0.0f,joints(3),convention);
	    H[4] = getDHMatrix(-0.0825f,-M_PI/2.0f,0.384f,joints(4),convention);
	    H[5] = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(5),convention);
	    H[6] = getDHMatrix(0.088f,M_PI/2.0f,0.107f,joints(6),convention);  
	    // H[7] = getDHMatrix(0.0f,0.0f,0.107f,0.0f,MODIFIED); 
	    Hee = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6];
	  }

	  // std::cerr << H[0] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1]*H[2] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1]*H[2]*H[3] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1]*H[2]*H[3]*H[4] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1]*H[2]*H[3]*H[4]*H[5] << std::endl << std::endl;
	  // std::cerr << H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6] << std::endl << std::endl;

	  Eigen::Matrix<T,3,1> xEE, z0, x0, xk, zk;

	  xEE = Hee.block(0,3,3,1);



	  Hk.setIdentity();

	  J.setConstant(0.0f);



	  for(int k = 0; k < 7; k++)
	  {

	    if(convention==MODIFIED)
	    {
	      Hk = Hk*H[k];
	    }
	    xk = Hk.block(0,3,3,1);
	    zk = Hk.block(0,2,3,1);

	    J.block(0,k,3,1) = zk.cross(xEE-xk);
	    J.block(3,k,3,1) = zk;
	    if(convention == NORMAL)
	    {
	      Hk = Hk*H[k];
	    }
	  }

	  J.block(0,0,3,7) += -getSkewSymmetricMatrix(rEEx)*J.block(3,0,3,7); 

	  return J;

	}

	// //////////////////////////////////////////////////////////////////////////////
	static Eigen::Matrix<T,4,4> pose2HomoMx(Eigen::Matrix<T,3,1> x, Eigen::Matrix<T,4,1> q)
	{
	  Eigen::Matrix<T,4,4> H; H.setIdentity();
	  H.block(0,3,3,1) = x;
	  Eigen::Quaternion<T> q_(q(0), q(1), q(2), q(3));
	  H.block(0,0,3,3) = q_.toRotationMatrix();

	  return H;
	}

	static Eigen::Matrix<T,3,3> getCombinedRotationMatrix(T weight, Eigen::Matrix<T,3,3> w_R_c, Eigen::Matrix<T,3,3> w_R_d)
	{
	    Eigen::Quaternion<T> qc(w_R_c);                       // current 
	    Eigen::Quaternion<T> qd(w_R_d);                       // desired 
	    Eigen::Quaternion<T> q_t   = qc.slerp(weight, qd);
	    Eigen::Matrix<T,3,3> w_R_cd_t = q_t.toRotationMatrix();
	    
	    return q_t.toRotationMatrix();
	}


	static Eigen::Matrix<T,4,1> getSlerpInterpolation(T weight, Eigen::Matrix<T,3,3> w_R_c, Eigen::Matrix<T,3,3> w_R_d)
	{
	    Eigen::Quaternion<T> qc(w_R_c);                       // current 
	    Eigen::Quaternion<T> qd(w_R_d);                       // desired 
	    Eigen::Quaternion<T> q_t   = qc.slerp(weight, qd);

	    Eigen::Matrix<T,4,1> q_out;
	    q_out << q_t.w(), q_t.x(), q_t.y(), q_t.z();
	    
	    return q_out;
	}

	static Eigen::Matrix<T,6,1> getPoseErrorCur2Des(Eigen::Matrix<T,4,4> d_H_c)
	{   
	    // Pass 
	    Eigen::Matrix<T,6,1> d_eta_c(6);
	    d_eta_c.segment(0,3) << d_H_c(0,3), d_H_c(1,3), d_H_c(2,3);
	    // extracrion of the rotation
	    Eigen::Matrix<T,3,3> d_R_c = d_H_c.block(0,0,3,3);
	    Eigen::AngleAxis<T> d_AxisAngle_c(d_R_c);
	    Eigen::Matrix<T,3,1> d_Axis_c = d_AxisAngle_c.axis();
	    d_eta_c(3) = d_Axis_c(0) * d_AxisAngle_c.angle();
	    d_eta_c(4) = d_Axis_c(1) * d_AxisAngle_c.angle();
	    d_eta_c(5) = d_Axis_c(2) * d_AxisAngle_c.angle();

	    return d_eta_c;
	}

	static Eigen::Matrix<T,3,1> getOrientationErrorCur2Des(Eigen::Matrix<T,3,3> d_R_c)
	{   
	    // Pass 
	    Eigen::Matrix<T,3,1> d_eta_c(3);
	    // extracrion of the rotation
	    Eigen::AngleAxis<T> d_AxisAngle_c(d_R_c);
	    Eigen::Matrix<T,3,1> d_Axis_c = d_AxisAngle_c.axis();
	    d_eta_c(0) = d_Axis_c(0) * d_AxisAngle_c.angle();
	    d_eta_c(1) = d_Axis_c(1) * d_AxisAngle_c.angle();
	    d_eta_c(2) = d_Axis_c(2) * d_AxisAngle_c.angle();

	    return d_eta_c;
	}

	static Eigen::Matrix<T,3,3> getMuThetaJacobian(Eigen::Matrix<T,3,3> d_R_c)
	{
	    // extracrion of the rotation
	    Eigen::AngleAxis<T> d_AxisAngle_c(d_R_c);
	    // function sinc(theta) and sinc(theta/2)
	    T sinc_theta, sinc_theta_2;
	    sinc_theta   = sin(d_AxisAngle_c.angle() + 1e-6)/(d_AxisAngle_c.angle() + 1e-6);
	    sinc_theta_2 = sin((d_AxisAngle_c.angle() + 1e-6)/2.)/((d_AxisAngle_c.angle() + 1e-6)/2.);
	    //
	    Eigen::Matrix<T,3,1> d_Axis_c = d_AxisAngle_c.axis();
	    Eigen::Matrix<T,3,3> Skew_Mu;    Skew_Mu.setZero(3,3);
	    //
	    Skew_Mu <<           0.0,   -d_Axis_c(2),    d_Axis_c(1),
	                 d_Axis_c(2),            0.0,   -d_Axis_c(0),
	                -d_Axis_c(1),    d_Axis_c(0),            0.0;

	    // Jacobian of the rotation
	    Eigen::Matrix<T,3,3> L_Mu_Theta;
	    L_Mu_Theta.setIdentity(3,3);
	    L_Mu_Theta = L_Mu_Theta - (d_AxisAngle_c.angle()/2.)* Skew_Mu + (1.-(sinc_theta/pow(sinc_theta_2, 2.))) * Skew_Mu * Skew_Mu;

	    return L_Mu_Theta;
	}

	static Eigen::Matrix<T,3,1> SaturationVect3(T lim, Eigen::Matrix<T,3,1> vel)
	{
	    Eigen::Matrix<T,3,1> v_ = vel;

	    if((fabs(vel(0))>lim) || (fabs(vel(1))>lim) || (fabs(vel(2))>lim)){
	        v_ = lim * (1./vel.norm() * vel);
	    }
	    return v_;
	}

	static Eigen::Matrix<T,6,1> SaturationTwist(T lim_l, T lim_a, Eigen::Matrix<T,6,1> vel)
	{
	    Eigen::Matrix<T,3,1> lin_, ang_;
	    lin_ = vel.head(3);
	    ang_ = vel.tail(3);

	    if((fabs(lin_(0))>lim_l) || (fabs(lin_(1))>lim_l) || (fabs(lin_(2))>lim_l)){
	        lin_ = lim_l * (1./lin_.norm() * lin_);
	    }
	    if((fabs(ang_(0))>lim_a) || (fabs(ang_(1))>lim_a) || (fabs(ang_(2))>lim_a)){
	        ang_ = lim_a * (1./ang_.norm() * ang_);
	    }

	    Eigen::Matrix<T,6,1> Vsat;
	    Vsat.head(3) = lin_;
	    Vsat.tail(3) = ang_;

	    return Vsat;
	}

	static Eigen::Matrix<T,12,12> getBimanualTaskTwistMapInv(T a_bi, T b_bi)
	{
	    // 
	    T a_bi_ = a_bi;
	    T b_bi_ = b_bi;
	    if(a_bi_!= 0.0)
	        b_bi_ = 1.0;
	    else if (a_bi_== 1.0)
	        b_bi_ = 0.0;

	    Eigen::Matrix<T, 12, 12> C_hands;
	    C_hands.setZero();
	    Eigen::Matrix<T, 6, 6> Idn;   Idn.setIdentity();
	    // Bimanual transformation
	    C_hands.topLeftCorner(6,6)     =               Idn;
	    C_hands.topRightCorner(6,6)    = -(1.-a_bi_) * Idn;
	    C_hands.bottomLeftCorner(6,6)  =      b_bi_  * Idn;
	    C_hands.bottomRightCorner(6,6) =       a_bi_ * Idn;
	    
	    return C_hands;
	}

	static void getBimanualTransforms(Eigen::Matrix<T,4,4> w_H_l, Eigen::Matrix<T,4,4> w_H_r, Eigen::Matrix<T,4,4> &w_H_a_, Eigen::Matrix<T,4,4> &l_H_r_)
	{
	  w_H_a_.setIdentity(4,4); 
	  l_H_r_.setIdentity(4,4); 
	  // relative transformation
	  // ========================
	  // l_H_r = W_H_l.inverse() * W_H_r;
	  l_H_r_.block(0,3, 3,1) = w_H_r.block(0,3, 3,1) - w_H_l.block(0,3, 3,1);       // translation expresse wrt. the world
	  l_H_r_.block(0,0, 3,3) = w_H_l.block(0,0, 3,3).transpose() * w_H_r.block(0,0, 3,3); // orienatation wrt. the left hand
	  // find the abolute transformation
	  // ======================================================
	  // Axis angle of relative hands orientation
	  Eigen::Matrix<T,3,3> l_R_r_ = l_H_r_.block(0,0, 3,3);
	  Eigen::AngleAxis<T> l_orientation_r(l_R_r_);
	  // Average orientation between hands
	  Eigen::Matrix<T,3,1> axis_ = l_orientation_r.axis();
	  T theta_  = 0.5*l_orientation_r.angle();
	  Eigen::AngleAxis<T> av_rot(theta_, axis_);

	  // Rotation matrix of the absolute hand frame expressed in the asbolute foot frame
	  Eigen::Matrix<T,3,3> W_R_a =  w_H_l.block(0,0, 3,3) * av_rot.toRotationMatrix();
	  w_H_a_.block(0,3, 3,1) = 0.5*(w_H_l.block(0,3, 3,1) + w_H_r.block(0,3, 3,1));

	  Eigen::Matrix<T,3,3> w_R_l = w_H_l.block(0,0, 3,3);
	  Eigen::Matrix<T,3,3> w_R_r = w_H_r.block(0,0, 3,3);
	  Eigen::Quaternion<T> qc(w_R_l);                       // current 
	  Eigen::Quaternion<T> qd(w_R_r);                       // desired 
	  T weight = 0.5;
	  Eigen::Quaternion<T> q_t   = qc.slerp(weight, qd);
	  Eigen::Matrix<T,3,3> w_R_cd_t = q_t.toRotationMatrix();
	    
	  w_H_a_.block(0,0, 3,3) = w_R_cd_t; //getCombinedRotationMatrix(weight, w_R_l, w_R_r);

	}

	static void getBimanualTwistDistribution(T a_bi, T b_bi, Eigen::Matrix<T,6,1> vel_a, Eigen::Matrix<T,6,1> vel_r, 
                                            Eigen::Matrix<T,6,1> &left_V, Eigen::Matrix<T,6,1> &right_V)
	{
	  Eigen::Matrix<T,12,12> Th = Utils<T>::getBimanualTaskTwistMapInv(a_bi, b_bi);
	  //
	  left_V  = Th.topLeftCorner(6,6)  *vel_a
	          + Th.topRightCorner(6,6) *vel_r;
	  //
	  right_V = Th.bottomLeftCorner(6,6)  *vel_a
	          + Th.bottomRightCorner(6,6) *vel_r;
	}

	static Eigen::Matrix<T,3,1> getEulerAnglesXYZ_FixedFrame(Eigen::Matrix<T,3,3> R)
  {
      // this function computed for a given rotation matrix the rotation angles around X, Y and Z axis considered as fixed.
      // the rotation matrix is assumed to be a Euler rotation matrix of type ZYX
      Eigen::Matrix<T,3,1> Angles;
      T Psi_X, Theta_Y, Phi_Z;
          Psi_X   = std::atan2(R(2,1),R(2,2));
          Theta_Y = std::atan2(-R(2,0), fabs(std::sqrt(std::pow(R(0,0), 2.)+std::pow(R(1,0), 2.))));
          Phi_Z   = std::atan2(R(1,0),R(0,0));
      if ((Theta_Y>M_PI/2.)||(Theta_Y<-M_PI/2.))
      {
          Psi_X   = std::atan2(-R(2,1),-R(2,2));
          Theta_Y = std::atan2(-R(2,0),-fabs(std::sqrt(std::pow(R(0,0), 2.)+std::pow(R(1,0), 2.))));
          Phi_Z   = std::atan2(-R(1,0),-R(0,0));
      }
      Angles(0) = Psi_X;
      Angles(1) = Theta_Y;
      Angles(2) = Phi_Z;

      return Angles;
  }

  static T computeCouplingFactor(Eigen::Matrix<T,3,1> ep_, T alpha_, T beta_, T gamma_, bool secondOrder)
	{
		T t_cpl_ = 1.0/(alpha_*ep_.norm()+1e-15);         
		T cpl_   = 0.0;
		t_cpl_ = pow(t_cpl_,gamma_);
		if(secondOrder)	cpl_   = 1.0 - exp(-t_cpl_/beta_) *(1.0 + t_cpl_/beta_);  // 2nd order critically damped
		else						cpl_   = 1.0 - exp(-t_cpl_/beta_); 												// 1st order increase

		return cpl_;
	}

	static Eigen::Matrix<T,3,3> create3dOrthonormalMatrixFromVector(Eigen::Matrix<T,3,1> inVec)
	{
	  //
	  int n = inVec.rows();
	  Eigen::Matrix<T,3,3> basis; // = Eigen::MatrixXf::Random(n,n); 
	  basis.setRandom(3,3);
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
	  	Eigen::Matrix<T,3,1> u = basis.col(0);
	  	Eigen::Matrix<T,3,1> v = basis.col(1);
	  	Eigen::Matrix<T,3,1> w = u.cross(v);
	  	basis.col(2) = w;
	  }
	  return basis;
	} 
	
	static void Orthobasis(Eigen::Matrix<T,3,1> v1,Eigen::Matrix<T,3,1> v0, Eigen::Matrix<T,3,3> &R1, Eigen::Matrix<T,3,3> &R0)
	{
	  Eigen::Matrix<T,3,1> i  = v1; 
	  Eigen::Matrix<T,3,1> id = v0; 

	  i.normalize();
	  id.normalize();
	  Eigen::Matrix<T,3,1> j  = i.cross(id);

	    if(j.norm() <1e-6){
        Eigen::Matrix<T,3,3> Rq = Utils<T>::create3dOrthonormalMatrixFromVector(i);
        j  = Rq.col(1);
      }

	    j.normalize();
	    Eigen::Matrix<T,3,1> jd = j;
	    Eigen::Matrix<T,3,1> k  = i.cross(j);
	    Eigen::Matrix<T,3,1> kd = id.cross(jd);
	    //
	    R1.col(0) = i;
	    R1.col(1) = j;
	    R1.col(2) = k;
	    //
	    R0.col(0) = id;
	    R0.col(1) = jd;
	    R0.col(2) = kd;
	  }

	static void UpdatePose_From_VelocityTwist(T dt, Eigen::Matrix<T,6,1> in_veloTwist, Eigen::Matrix<T,4,4> &Hmg_Trsf)
	{
	    //
	    Eigen::Matrix<T,3,1> pos = Hmg_Trsf.block(0,3,3,1);
	    Eigen::Matrix<T,3,3> rot = Hmg_Trsf.block(0,0,3,3);

	    // update position
	    pos = pos + dt * in_veloTwist.head(3);

	    // update orientation
	    Eigen::Quaternion<T> q(rot);

	    Eigen::Matrix<T,4,3> Trf_quat;
	    Trf_quat << -q.x(), -q.y(), -q.z(),
	                 q.w(),  q.z(), -q.y(),
	                -q.z(),  q.w(),  q.x(),
	                 q.y(), -q.x(),  q.w(); 
	    // update the quaternion
	    Eigen::Matrix<T,4,1> qcoeff;
	    qcoeff << q.w(), q.x(), q.y(), q.z();
	    qcoeff =  qcoeff + dt* 0.5 *Trf_quat * in_veloTwist.tail(3);

	    // normalizing the quaternion
	    qcoeff.normalize();
	    if(qcoeff.norm() <= 1e-8){
	      qcoeff = Eigen::Matrix<T,4,1>(1.0, 0.0, 0.0, 0.0);
	    }
	    //
	    Eigen::Quaternion<T> q_new(qcoeff(0), qcoeff(1), qcoeff(2), qcoeff(3)); // w, x, y, z
	    //
	    Hmg_Trsf.setZero();
	    //
	    Hmg_Trsf.block(0,0,3,3) = q_new.toRotationMatrix(); //rot; //
	    Hmg_Trsf.block(0,3,3,1) = pos;
	    Hmg_Trsf(3,3) = 1.0;
	}

	static void UpdatePose_From_VelocityTwist(T dt, Eigen::Matrix<T,6,1> in_veloTwist, Eigen::Matrix<T,3,1> &curPos, Eigen::Matrix<T,4,1> &curOrient)
	{
	    //
	    Eigen::Matrix<T,4,4> cur_Hmg_Trsf = Utils<T>::pose2HomoMx(curPos, curOrient);

	    // update position
	    curPos = curPos + dt * in_veloTwist.head(3);

	    // update orientation
	    Eigen::Matrix<T,4,3> Trf_quat;
	    Trf_quat << -curOrient(1), -curOrient(2), -curOrient(3),
	                 curOrient(0),  curOrient(3), -curOrient(2),
	                -curOrient(3),  curOrient(0),  curOrient(1),
	                 curOrient(2), -curOrient(1),  curOrient(0); 
	    // update the quaternion
	    curOrient = curOrient + dt* 0.5 *Trf_quat * in_veloTwist.tail(3);

	    // normalizing the quaternion
	    curOrient.normalize();
	    if(curOrient.norm() <= 1e-8){
	      curOrient = Eigen::Matrix<T,4,1>(1.0, 0.0, 0.0, 0.0);
	    }
	}

	static Eigen::Matrix<T,3,1> cartesian2planar(Eigen::Matrix<T,3,1> Pos_d){

		Eigen::Matrix<T,3,1> out;
		out.setZero();
		out(0) = Pos_d.head(2).norm();  									// r
	  out(1) = Pos_d(2);																// z
	  out(2) = std::atan2(Pos_d(1), Pos_d(0));					// phi
		return out;
	}

	static Eigen::Matrix<T,3,1> cartesian2spherical(Eigen::Matrix<T,3,1> Pos_d){
		//
		Eigen::Matrix<T,3,1> out;
		out.setZero();
		out(0) = Pos_d.norm(); 																// r
	  // out(1) = std::acos( Pos_d(2), out(0));							// theta
	  out(1) = std::atan2( Pos_d.head(2).norm(), Pos_d(2)); // theta				
	  out(2) = std::atan2(Pos_d(1), Pos_d(0));							// phi

		return out;
	}

	static Eigen::Matrix<T,3,1> get_abs_3d(Eigen::Matrix<T,3,1> v_left, Eigen::Matrix<T,3,1> v_right){
		return 0.5 *(v_left+v_right);
	}
	static Eigen::Matrix<T,3,1> get_abs_3d(Eigen::Matrix<T,4,4> H_left, Eigen::Matrix<T,4,4> H_right){
		return 0.5 *(H_left.block(0,3,3,1) + H_right.block(0,3,3,1));
	}
		
	static Eigen::Matrix<T,3,1> get_abs_3d(Eigen::Matrix<T,3,1> Vec[2]){
		return 0.5 *(Vec[0]+Vec[1]);
	}

	static Eigen::Matrix<T,3,1> get_abs_3d(Eigen::Matrix<T,6,1> Vec[2], bool top){
		//
		if(!top){
			return 0.5 *(Vec[0].tail(3)+Vec[1].tail(3));
		}
		else{
			return 0.5 *(Vec[0].head(3)+Vec[1].head(3));
		}
	}				

	static Eigen::Matrix<T,3,1> get_abs_3d(Eigen::Matrix<T,4,4> H[2]){
		return 0.5 *(H[0].block(0,3,3,1)+H[1].block(0,3,3,1));
	}		


};

template class Utils<float>;
template class Utils<double>;

class MatrixPseudoInverse2
{

    public : 

        MatrixPseudoInverse2(){}

        ~MatrixPseudoInverse2(){}

        // Compute the pseudo inverse of a matrix
        template<typename _Matrix_Type_> _Matrix_Type_ get_pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
        {
            
            Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);

            int svdSize = svd.singularValues().size();

            double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

            return svd.matrixV().leftCols(svdSize) *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().leftCols(svdSize).adjoint();
        }

        bool get_HhQRPseudoInverse(Eigen::MatrixXd myMatrix, Eigen::MatrixXd &PsdInvmyMatrix)
        {

            Eigen::HouseholderQR<Eigen::MatrixXd> qr(myMatrix.transpose());
            PsdInvmyMatrix.setIdentity(myMatrix.cols(), myMatrix.rows());
            PsdInvmyMatrix = qr.householderQ() * PsdInvmyMatrix;
            PsdInvmyMatrix = qr.matrixQR().topLeftCorner(myMatrix.rows(),myMatrix.rows()).triangularView<Eigen::Upper>().transpose().solve<Eigen::OnTheRight>(PsdInvmyMatrix);

            return true;

        }

        bool get_CODecomPseudoInverse(Eigen::MatrixXd myMatrix, Eigen::MatrixXd &PsdInvmyMatrix)
        {
            //
            Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(myMatrix);
            PsdInvmyMatrix = cqr.pseudoInverse();

            return true;
        }

        bool get_LLTSolveInverse(Eigen::MatrixXd myMatrix, Eigen::MatrixXd &Inv_myMatrix)
        {
            //
            Eigen::MatrixXd UnitMx = Eigen::MatrixXd::Identity(myMatrix.cols(), myMatrix.rows()); 
            Inv_myMatrix = myMatrix.llt().solve(UnitMx);

            return true;
        }

        bool get_LUSolveInverse(Eigen::MatrixXd myMatrix, Eigen::MatrixXd &Inv_myMatrix)
        {
            //
            Eigen::MatrixXd UnitMx = Eigen::MatrixXd::Identity(myMatrix.cols(), myMatrix.rows()); 
            Inv_myMatrix = myMatrix.lu().solve(UnitMx);

            return true;
        }
        bool get_LDLTSolveInverse(Eigen::MatrixXd myMatrix, Eigen::MatrixXd &Inv_myMatrix)
        {
            //
            Eigen::MatrixXd UnitMx = Eigen::MatrixXd::Identity(myMatrix.cols(), myMatrix.rows()); 
            Inv_myMatrix = myMatrix.ldlt().solve(UnitMx);

            return true;
        }

};

// Kalman filter
class KalmanFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanFilter(   float dt,
                  const Eigen::MatrixXf& A,
                  const Eigen::MatrixXf& C,
                  const Eigen::MatrixXf& Q,
                  const Eigen::MatrixXf& R,
                  const Eigen::MatrixXf& P
                                            ): A(A), C(C), Q(Q), R(R), P0(P),
                                               m(C.rows()), n(A.rows()), dt(dt), initialized(false),
                                               I(n, n), x_hat(n), x_hat_new(n)
    {
      I.setIdentity();
    }

  /**
  * Create a blank estimator.
  */
  KalmanFilter(){}

  /**
  * Initialize the filter with initial states as zero.
  */
  void init(){
      x_hat.setZero();
      P = P0;
      t0 = 0;
      t = t0;
      initialized = true;
    }

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(float t0, const Eigen::VectorXf& x0){
      x_hat = x0;
      P = P0;
      this->t0 = t0;
      t = t0;
      initialized = true;
    }
  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const Eigen::VectorXf& y){

      if(!initialized)
        throw std::runtime_error("Filter is not initialized!");

      x_hat_new = A * x_hat;
      P = A*P*A.transpose() + Q;
      K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
      x_hat_new += K * (y - C*x_hat_new);
      P = (I - K*C)*P;
      x_hat = x_hat_new;

      t += dt;
    }
  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(const Eigen::VectorXf& y, float dt, const Eigen::MatrixXf A){

      this->A = A;
      this->dt = dt;
      update(y);
    }
  /**
  * Return the current state and time.
  */
  Eigen::VectorXf state() { return x_hat; };
  float time() { return t; };

  void setState(const Eigen::VectorXf& x_hat_1)
  {
        x_hat = x_hat_1;
  }

private:

  // Matrices for computation
  Eigen::MatrixXf A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  float t0, t;

  // Discrete time step
  float dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXf I;

  // Estimated states
  Eigen::VectorXf x_hat, x_hat_new;
};

//
class KF_3DVeloFromPosEstimator
{
	public:
		KF_3DVeloFromPosEstimator(){};
		~KF_3DVeloFromPosEstimator(){};

		std::unique_ptr<KalmanFilter> _KFilter[3];

		void init(float dt, Eigen::Vector2f Q, float R, Eigen::Vector3f Xm)
		{

			int n = 2; // Number of states
			int m = 1; // Number of measurements

			float sigmaQ1 = Q(0); //0.004;  // 0.05
			float sigmaQ2 = Q(1); //0.07;
			float sigmaR  = R; 		//0.07;  // 2.00

			Eigen::MatrixXf A2(n, n); // System dynamics matrix
			Eigen::MatrixXf C2(m, n); // Output matrix
			Eigen::MatrixXf Q2(n, n); // Process noise covariance
			Eigen::MatrixXf R2(m, m); // Measurement noise covariance
			Eigen::MatrixXf P2(n, n); // Estimate error covariance
			Eigen::Matrix<float, 2, 1> X_k0[3]; // state vector Y_k0(2), Z_k0(2);

			A2 << 1, dt, 0, 1;
			C2 << 1, 0;
			// Reasonable covariance matrices
			Q2(0,0) =  sigmaQ1*sigmaQ1*dt*dt*dt/3.;   Q2(0,1) =  sigmaQ1*sigmaQ2*dt*dt/2.;
			Q2(1,0) =  sigmaQ1*sigmaQ2*dt*dt/2.;      Q2(1,1) =  sigmaQ2*sigmaQ2*dt;

			P2(0,0) =  sigmaR*sigmaR;             P2(0,1) =  sigmaR*sigmaR/(2.*dt);
			P2(1,0) =  sigmaR*sigmaR/(2.*dt);     P2(1,1) =  2./3.*sigmaQ2*sigmaQ2*dt+sigmaR*sigmaR/(2.*dt*dt);

			R2 << sigmaR;

			for(int i=0; i<3; i++){
				_KFilter[i] = std::make_unique<KalmanFilter>(dt, A2, C2, Q2, R2, P2);
				X_k0[i](0) = Xm(i);
				X_k0[i](1) = 0.0;
				_KFilter[i]->init(0., X_k0[i]);
			}
		}

		/**
	  * Update the estimated state based on measured values. The
	  * time step is assumed to remain constant.
	  */
	  void update(const Eigen::Vector3f& Xm){
	  	//
	  	Eigen::Matrix<float, 1, 1> X_k[3];
	  	//
	  	for(int i=0; i<3; i++){
	  		X_k[i](0) = Xm(i);
	  		_KFilter[i]->update(X_k[i]);
	  	}
	  }

	  Eigen::Vector3f get_estimate_position(){

			Eigen::Vector3f X_hat;
			for(int i=0; i<3; i++){
	  		X_hat(i) = (_KFilter[i]->state())(0);
	  	}
	  	return X_hat;
	  }

	  Eigen::Vector3f  get_estimate_velocity(){
	  	Eigen::Vector3f dX_hat;
			for(int i=0; i<3; i++){
	  		dX_hat(i) = (_KFilter[i]->state())(1);
	  	}
	  	return dX_hat;
	  }

};

namespace utils{
	class firstOrderFilter
	{
	    double Ts;

	    // Eigen::VectorXd init_fn;
	    Eigen::VectorXd init_fn2;
	    Eigen::VectorXd init_fn3;
	    Eigen::VectorXd init_fn4;
	    Eigen::VectorXd delta1;
	    Eigen::VectorXd delta2;
	    Eigen::VectorXd delta3;
	    Eigen::VectorXd delta4;
	    Eigen::VectorXd y_t;

		public:

	    double pole;
	    double gain;
	    Eigen::VectorXd init_fn;

	    firstOrderFilter(){}
	    // 
	    void InitializeFilter(double T, double gn, double pl, Eigen::VectorXd init_fn_val)
	    {
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

	     ~firstOrderFilter(){}

	    Eigen::VectorXd function_dot(double gn, double pl, const Eigen::VectorXd &init_fn_val, const Eigen::VectorXd &fn_t)
	    {
	        return - pl * init_fn_val + gn * fn_t;
	    }

	    // compute the integral of first order differential eq. using RK4
	    Eigen::VectorXd getRK4Integral(const Eigen::VectorXd &fn_t)
	    {
	      delta1   = Ts * function_dot(gain, pole, init_fn, fn_t);
	      init_fn2 = init_fn + 0.5 * delta1;
	      delta2   = Ts * function_dot(gain, pole, init_fn2, fn_t);
	      init_fn3 = init_fn + 0.5 * delta2;
	      delta3   = Ts * function_dot(gain, pole, init_fn3, fn_t);
	      init_fn4 = init_fn + 0.5 * delta3;
	      delta4   = Ts * function_dot(gain, pole, init_fn4, fn_t);

	      // solution
	      y_t      = init_fn + 1/6. * (delta1 + 2.* delta2 + 2.* delta3 + delta4);
	      init_fn  = y_t;

	      return y_t;
	    }

	    Eigen::VectorXd getEulerIntegral(const Eigen::VectorXd &fn_t)
	    {
	      delta1   = Ts * function_dot(gain, pole, init_fn, fn_t);
	      // solution
	      y_t      = init_fn + delta1;
	      init_fn  = y_t;

	      return y_t;

	    }

	    void setGain(double _gain){
	        gain = _gain;
	    }

	    void setPole(double _pole){
	        pole = _pole;
	    }

	    void setSampleTime(double T){
	        Ts = T;
	    }   
	};
}
	
template<typename T = float>
class DataLoader{

	public: 

		DataLoader(){}
		~DataLoader(){};

	// function to log data from file
		bool LoadDataFromFile(std::string file_name, Eigen::Matrix<T, Eigen::Dynamic, 1> &data_all_val)
		{
		    //
		    std::ifstream inFile;
		    inFile.open(file_name);
		    if (!inFile) {
		        std::cout << "Unable to open file \n";
		        exit(1);                            // terminate with error
		    }
		    //
		    std::vector<T> data_val;
		    T x; 
		    //
		    while(inFile >> x) {
		        data_val.push_back(x);
		    }
		    //
		    int size_data_val = data_val.size();
		    //
		    data_all_val.resize(size_data_val);
		    for(int i=0; i<size_data_val; i++) 
		        data_all_val(i) = data_val[i];

		    return true;
		}


		// 
		bool Load_gmm_param(std::string file_name[], int dataDim, int nbStates, 
												Eigen::Matrix<T, Eigen::Dynamic, 1> &Priors_, 
												Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &Means_, 
												Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &Covars_)
		{

			// 
			std::string Priors_file_name  = file_name[0]; // + "_prio.txt";  
			std::string Means_file_name   = file_name[1]; // + "_mu.txt";  
			std::string Covar_file_name   = file_name[2]; // + "_sigma.txt";  
			//
			Eigen::Matrix<T, Eigen::Dynamic, 1> priors_all_val;
			Eigen::Matrix<T, Eigen::Dynamic, 1> means_all_val;
			Eigen::Matrix<T, Eigen::Dynamic, 1> covars_all_val;
			//
			DataLoader<T>::LoadDataFromFile(Priors_file_name, priors_all_val);
			DataLoader<T>::LoadDataFromFile(Means_file_name,  means_all_val);
			DataLoader<T>::LoadDataFromFile(Covar_file_name,  covars_all_val);
			//
			// Priors
			Priors_ = priors_all_val;
			// Means
			Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Means_Mx(means_all_val.data(),dataDim, nbStates);
			Means_ = Means_Mx;
			
			//
			int row_cov = dataDim * nbStates;
			// Covariance
			Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Covar_Mx(covars_all_val.data(),row_cov,dataDim);
			Covars_ = Covar_Mx;


			return true;
		}


		bool Load_gmm_param2(std::string file_name[], Eigen::Matrix<T, Eigen::Dynamic, 1> &Priors_, 
												Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &Means_, 
												Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &Covars_)
		{
			// 
			std::string Priors_file_name  = file_name[0]; // + "_prio.txt";  
			std::string Means_file_name   = file_name[1]; // + "_mu.txt";  
			std::string Covar_file_name   = file_name[2]; // + "_sigma.txt";  
			//
			Eigen::Matrix<T, Eigen::Dynamic, 1>  priors_all_val;
			Eigen::Matrix<T, Eigen::Dynamic, 1>  means_all_val;
			Eigen::Matrix<T, Eigen::Dynamic, 1>  covars_all_val;
			//
			DataLoader<T>::LoadDataFromFile(Priors_file_name, priors_all_val);
			DataLoader<T>::LoadDataFromFile(Means_file_name,  means_all_val);
			DataLoader<T>::LoadDataFromFile(Covar_file_name,  covars_all_val);
			//
			// Priors
			Priors_  = priors_all_val;
			//
			int nbStates = priors_all_val.rows();
			int dataDim  = int(means_all_val.rows()/nbStates);
			// Means
			Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Means_Mx(means_all_val.data(),dataDim, nbStates);
			Means_ = Means_Mx;

			//
			int row_cov = dataDim * nbStates;
			// Covariance
			Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Covar_Mx(covars_all_val.data(),row_cov,dataDim);
			Covars_ = Covar_Mx;


			return true;
		}
};

template class DataLoader<float>;
template class DataLoader<double>;

#endif
