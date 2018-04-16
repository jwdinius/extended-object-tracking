/*
  ==============================================================================

    Kf.cpp
    Created: 24 Feb 2018 8:14:38am
    Author:  Joseph Dinius

  ==============================================================================
*/

#include "Kf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Kalman filter
 */
KF::KF() {
  // initial covariance matrices
  // state
  Matrix2d eye;
  eye.setIdentity();
  Cr_.setZero();
  Cr_.topLeftCorner(2, 2) = INIT_VAR_POS*eye;
  Cr_.bottomRightCorner(2, 2) = INIT_VAR_VEL*eye;
  // shape/orientation
  Cp_.setZero();
  Cp_(0,0) = INIT_VAR_ALPHA;
  Cp_.bottomRightCorner(2, 2) = INIT_VAR_LENGTH*eye;
  
  // process covariances
  // state
  Cwr_.setZero();
  Cwr_.topLeftCorner(2, 2) = VAR_POS*eye;
  Cwr_.bottomRightCorner(2, 2) = VAR_VEL*eye;
  // shape/orientation
  Cwp_.setZero();
  Cwp_(0,0) = VAR_ALPHA;
  Cwp_.bottomRightCorner(2, 2) = VAR_LENGTH*eye;
  
  // measurement covariances
  Cv_ << VAR_V1  ,    0   ,
           0     ,  VAR_V2;
  
  Ch_ << VAR_H   ,    0   ,
           0     ,  VAR_H ;

  /**
  Other initializations
  */
  is_initialized_ = false;
  dt_             = 10;
  // state transition matrix - cartesian
  Ar_ << 1 , 0, dt_, 0,
         0 , 1, 0, dt_,
         0 , 0, 1,  0 ,
         0 , 0, 0,  1 ;
  
  // shape/orientation transition matrix
  Ap_.setIdentity();
  
  // pseudomeasurement transformation (measure cartesian position)
  H_ = MatrixXd(2,4);
  H_ << 1, 0, 0, 0,
        0, 1, 0, 1;
}

KF::~KF() {}

/**
 * ProcessMeasurement(meas)
 * handles measurement struct passed from sensor
 */
void KF::ProcessMeasurement(SensorUdpTelemetry meas) {
  if (!is_initialized_) {
    /**
    * if the filter hasn't been initialized yet,
    * set the position state to the mean of all valid
    * detections and zero out the velocity
    */
    
    double mx = 0;
    double my = 0;
    int n = 0;
    for (int i = 0; i < MAX_DETS; i++) {
        if (meas.posX[i] < std::numeric_limits<double>::infinity()){
            mx += meas.posX[i];
            my += meas.posY[i];
            n  += 1;
        }
    }
    
    mx /= double(n);
    my /= double(n);
    
    r_ <<  mx ,
           my ,
           0  ,
           0  ;
    
    // assume that the initial shape is a circle with some a priori knowledge about
    // the extended object
    p_ <<     0     ,
           OBJECT_W ,
           OBJECT_W ;
    
    // set filter initialized
    is_initialized_ = true;
    
    return;
  }

  // predict step
  Prediction();
  
  // update step
  Update(meas);
  
}

/**
 * Standard KF state and covariance prediction
 */
void KF::Prediction() {
  /**
  * This assumes a nearly-constant velocity prediction model
  */
  // state k|k-1
  r_ = Ar_ * r_;
  // shape/orientation k|k-1
  p_ = Ap_ * p_;
  // state covariance k|k-1
  Cr_ = Ar_ * Cr_ * Ar_.transpose() + Cwr_;
  // shape/orientation covariance k|k-1
  Cp_ = Ap_ * Cp_ * Ap_.transpose() + Cwp_;
}

void KF::Update(SensorUdpTelemetry meas){
  /**
  * For each valid measurement, ...
  */
  for (int i = 0; i < MAX_DETS; i++){
    if (meas.posX[i] < std::numeric_limits<double>::infinity()){
      // current measurement
      Vector2d y;
      y << meas.posX[i] ,
           meas.posY[i] ;
      
      // compute S and M for state and covariance updates
      Matrix2d Ss;
      Matrix2d Rot;
      Rot << cos(p_(0)) , -sin(p_(0)) ,
             sin(p_(0)) ,  cos(p_(0)) ;
      Matrix2d D;
      D << p_(1) ,    0   ,
             0   , p_(2) ;
      Ss = Rot * D;
  
      Matrix3d M1;
      Matrix3d M2;
      Matrix3d M;
      double cp  = cos(p_(0));
      double c2p = cos(2.*p_(0));
      double sp  = sin(p_(0));
      double s2p = sin(2.*p_(0));
      double c11  = p_(1)*p_(1)*Ch_(0,0) - p_(2)*p_(2)*Ch_(1,1);
      double c22  = 2.*p_(1)*Ch_(0,0);
      double c33  = 2.*p_(2)*Ch_(1,1);
  
      M1 <<   -s2p   ,   cp*cp  ,   sp*sp   ,
               c2p   ,    s2p   ,   -s2p    ,
               s2p   ,   sp*sp  ,   cp*cp   ;
      M2 <<    c11   ,     0    ,     0     ,
                0    ,    c22   ,     0     ,
                0    ,     0    ,    c33    ;
            
      M = M1 * M2;
      
      // pseudomeasurement
      Vector2d E_y;
      E_y = H_ * r_;
      // moments of the kinematic state
      MatrixXd C_ry = MatrixXd(4, 2);
      C_ry = Cr_ * H_.transpose();
      Matrix2d C_yy;
      C_yy = H_ * Cr_ * H_.transpose() + Ss * Ch_ * Ss.transpose() + Cv_;
      // measurement residual
      Vector2d res;
      res = y - E_y;
      r_ = r_ + C_ry * C_yy.inverse() * res;
      Cr_ = Cr_ - C_ry * C_yy.inverse() * C_ry.transpose();
      // force symmetry
      Cr_ = (Cr_ + Cr_.transpose()) / 2;
      
      // the shape/orientation follows
      MatrixXd T = MatrixXd(3,4);
      T.setZero();
      T(0,0) = 1;
      T(1,1) = 1;
      T(2,3) = 1;
      
      // kronecker product for quadratic measurement model (see paper)
      Vector4d kron;
      kron << res(0) * res(0) ,
              res(0) * res(1) ,
              res(1) * res(0) ,
              res(1) * res(1) ;
      Vector3d Y;
      Y = T*kron;
      
      Vector3d E_Y;
      E_Y << C_yy(0,0) ,
             C_yy(0,1) ,
             C_yy(1,1) ;
      
      Matrix3d C_YY;
      C_YY <<          3 * E_Y(0)*E_Y(0)       ,           3 * E_Y(0)*E_Y(1)     ,  E_Y(0)*E_Y(2) + 2*E_Y(1)*E_Y(1) ,
                       3 * E_Y(0)*E_Y(1)       , E_Y(0)*E_Y(2) + 2*E_Y(1)*E_Y(1) ,            3*E_Y(2)*E_Y(1)       ,
               E_Y(0)*E_Y(2) + 2*E_Y(1)*E_Y(1) ,           3 * E_Y(2)*E_Y(1)     ,            3*E_Y(2)*E_Y(2)       ;
               
      // moments of the shape/orientation
      Matrix3d C_pY;
      C_pY = Cp_ * M.transpose();
      
      // shape updates
      p_  = p_  + C_pY * C_YY.inverse() * (Y-E_Y);
      Cp_ = Cp_ - C_pY * C_YY.inverse() * C_pY.transpose();
      // force symmetry
      Cp_ = (Cp_ + Cp_.transpose()) / 2;
      
    }
  }
}

/**
 * handle angle wraparound (currently unused)
 */
void KF::NormalizeAngle(double &angle){
  while (angle> M_PI) angle-=2.*M_PI;
  while (angle<-M_PI) angle+=2.*M_PI;
}

/**
 * return state and shape estimates for plotting
 */
VectorXd KF::getState(){
  VectorXd out = VectorXd( r_.size() + p_.size() );
  out.head(r_.size()) = r_;
  out.tail(p_.size()) = p_;
  
  return out;
}

/**
 * for debugging, print outputs
 */
void KF::printOutput(){
  std::cout << r_ << std::endl;
  std::cout << "" << std::endl;
  std::cout << p_ << std::endl;
  std::cout << "" << std::endl;
  std::cout << Cr_ << std::endl;
  std::cout << "" << std::endl;
  std::cout << Cp_ << std::endl;
  std::cout << "" << std::endl;
}
