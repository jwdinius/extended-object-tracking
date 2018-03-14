/*
  ==============================================================================

    Ukf.cpp
    Created: 24 Feb 2018 8:14:38am
    Author:  Joseph Dinius

  ==============================================================================
*/

#include "Ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    
  // 

  // initial state vectors
  x_ = VectorXd(4);
  /*x_ << -3900 ,
         100  , 
         5    ,
        -8    ;*/
  p_ = VectorXd(3);
  /*p_ << -M_PI/3. ,
         200     ,
          90     ;*/

  // initial covariance matrice
  Px_ = MatrixXd(4, 4);
  Px_ <<   100  ,  0  ,  0  ,  0  ,
            0   , 100 ,  0  ,  0  ,
            0   ,  0  ,  1  ,  0  ,
            0   ,  0  ,  0  ,  1  ;
  Pp_ = MatrixXd(3, 3);
  Pp_ <<  .04  ,    0    ,    0    ,
           0   ,   0.5   ,    0    ,
           0   ,    0    ,   0.5   ;
  
  // measurement covariances
  Rx_ = MatrixXd(2, 2); //C_v
  Rx_ << STD_POSX * STD_POSX  ,          0         ,
                  0           , STD_POSY * STD_POSY;
  Rx_ *= 0.2;
  Rp_ = MatrixXd(2, 2); //C_h
  Rp_ << STD_H * STD_H ,       0      ,
               0       , STD_H * STD_H;
  
  // process covariances
  Qx_ = MatrixXd(2, 2);
  Qx_ << STD_A * STD_A ,        0      ,
               0       ,  STD_A * STD_A;
  Qp_ = MatrixXd(3, 3);
  Qp_ << STD_ALPHA * STD_ALPHA ,        0        ,        0        ,
                   0           ,  STD_L * STD_L  ,        0        ,
                   0           ,        0        ,  STD_L * STD_L  ;


  /**
  TODO:
  Complete the initialization. See ukf.h for other member properties.
  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  time_           = 0;
  n_augx_         = 6;
  n_augp_         = 6;
  n_x_            = 4;
  n_p_            = 3;
  nisx_           = 0;
  nisp_           = 0;
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_augx_ + 1);
  Psig_pred_ = MatrixXd(n_p_, 2 * n_augp_ + 1);
  weightsx_ = VectorXd(2 * n_augx_ + 1);
  weightsp_ = VectorXd(2 * n_augp_ + 1);
  lambdax_ = 3 - n_augx_;
  lambdap_ = 3 - n_augp_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(SensorUdpTelemetry meas) {
  /**
  TODO:
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
    */
    
    // set to mean of detections
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
    
    mx /= float(n);
    my /= float(n);
    
    x_ <<  mx ,
           my ,
           0  ,
           0  ;
    
    p_ <<     0     ,
           OBJECT_W ,
           OBJECT_W ;
    
    // done initializing, no need to predict or update
    time_ = meas.timestamp;
    is_initialized_ = true;
    weightsx_.fill(0);
    weightsp_.fill(0);
    Xsig_pred_.fill(0);
    Psig_pred_.fill(0);
  
    for (unsigned i = 0; i < 2*n_augx_+1; i++){
      if (i == 0){
        weightsx_(i) = lambdax_ / (lambdax_ + double(n_augx_));
      }
      else{
        weightsx_(i) = 1. / (2. * (lambdax_ + double(n_augx_)));
      }
    }
    
    for (unsigned i = 0; i < 2*n_augp_+1; i++){
      if (i == 0){
        weightsp_(i) = lambdap_ / (lambdap_ + double(n_augp_));
      }
      else{
        weightsp_(i) = 1. / (2. * (lambdap_ + double(n_augp_)));
      }
    }
    return;
  }

  double delta_t = meas.timestamp - time_;
  
  // predict
  Prediction(delta_t);
  
  // update
  Update(meas);
  
  time_ = meas.timestamp;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //create augmented mean vectors
  VectorXd x_aug = VectorXd(n_augx_);
  x_aug.fill(0.);
  VectorXd p_aug = VectorXd(n_augp_);
  p_aug.fill(0.);

  //create augmented state covariances
  MatrixXd Px_aug = MatrixXd(n_augx_, n_augx_);
  Px_aug.fill(0.);
  MatrixXd Pp_aug = MatrixXd(n_augp_, n_augp_);
  Pp_aug.fill(0.);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_augx_, 2 * n_augx_ + 1);
  Xsig_aug.fill(0.);
  MatrixXd Psig_aug = MatrixXd(n_augp_, 2 * n_augp_ + 1);
  Psig_aug.fill(0.);
  

  //create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(4) = 0.;
  x_aug(5) = 0.;
  p_aug.head(n_x_) = x_;
  p_aug(3) = 0.;
  p_aug(4) = 0.;
  p_aug(5) = 0.;
  
  //create augmented covariance matrix
  Px_aug.topLeftCorner(n_x_,n_x_) = Px_;
  Px_aug.bottomRightCorner(2,2)   = Qx_;
  
  //create square root matrix
  MatrixXd Ax = Px_aug.llt().matrixL();
  MatrixXd Ap = Pp_aug.llt().matrixL();
  
  //create augmented sigma points
  //set sigma points as columns of matrix Xsig
  // first point is the mean
  Xsig_aug.col(0) = x_aug;
  Psig_aug.col(0) = p_aug;
  
  // now, for the rest, i and i+n_x can be filled via a loop:
  for (unsigned i=1; i <= n_augx_; i++){
    Xsig_aug.col(i)         = x_aug + sqrt(lambdax_+double(n_augx_))*Ax.col(i-1);
    Xsig_aug.col(i+n_augx_) = x_aug - sqrt(lambdax_+double(n_augx_))*Ax.col(i-1);
  }
  
  for (unsigned i=1; i <= n_augp_; i++){
    Psig_aug.col(i)         = p_aug + sqrt(lambdap_+double(n_augp_))*Ap.col(i-1);
    Psig_aug.col(i+n_augp_) = p_aug - sqrt(lambdap_+double(n_augp_))*Ap.col(i-1);
  }

  //predict sigma points - state
  for (unsigned i = 0; i < Xsig_pred_.cols(); i++){
    VectorXd y    = Xsig_aug.col(i);
    VectorXd x    = y.head(n_x_);
    double dt2    = delta_t*delta_t;
    VectorXd x_up = VectorXd(n_x_);
    
    x_up(0) = y(2) * delta_t + 0.5 * dt2 * y(4);
    x_up(1) = y(3) * delta_t + 0.5 * dt2 * y(5);
    x_up(2) = delta_t * y(4);
    x_up(3) = delta_t * y(5);
      
    //write predicted sigma points into right column
    Xsig_pred_.col(i) = x + x_up;
  }
  
  //predict sigma points - shape
  for (unsigned i = 0; i < Psig_pred_.cols(); i++){
    VectorXd y    = Psig_aug.col(i);
    VectorXd p    = y.head(n_p_);
    VectorXd p_up = VectorXd(n_p_);
    
    p_up(0) = y(3);
    p_up(1) = y(4);
    p_up(2) = y(5);
      
    //write predicted sigma points into right column
    Psig_pred_.col(i) = p + p_up;
  }
  

  //predict state mean
  x_.fill(0.);
  for (unsigned i = 0; i < 2*n_augx_+1; i++){
    x_ += weightsx_(i) * Xsig_pred_.col(i);
  }
  // predict mean - shape
  p_.fill(0.);
  for (unsigned i = 0; i < 2*n_augp_+1; i++){
    p_ += weightsp_(i) * Psig_pred_.col(i);
  }

  //predict state covariance matrix
  Px_.fill(0.);
  VectorXd res = VectorXd(n_x_);
  for (unsigned i = 0; i < 2*n_augx_+1; i++){
      res = Xsig_pred_.col(i) - x_;  
      Px_ += weightsx_(i) * res * res.transpose();
  }
  Px_ = (Px_ + Px_.transpose()) / 2;
  
  //predict covariance matrix - shape
  Pp_.fill(0.);
  VectorXd resp = VectorXd(n_p_);
  for (unsigned i = 0; i < 2*n_augp_+1; i++){
      resp = Psig_pred_.col(i) - p_;
      Pp_ += weightsp_(i) * resp * resp.transpose();
  }
  Pp_ = (Pp_ + Pp_.transpose()) / 2;
  
}

void UKF::Update(SensorUdpTelemetry meas){
  nisx_ = 0;
  nisp_ = 0;
  
  for (int i = 0; i < MAX_DETS; i++){
    if (meas.posX[i] < std::numeric_limits<double>::infinity()){
      MatrixXd Ss = MatrixXd(2,2);
      MatrixXd Rot = MatrixXd(2,2);
      Rot << cos(p_(0)) , -sin(p_(0)) ,
             sin(p_(0)) ,  cos(p_(0)) ;
      MatrixXd D = MatrixXd(2,2);
      D << p_(1) ,    0   ,
             0   , p_(2) ;
      Ss = Rot * D;
  
      MatrixXd M1 = MatrixXd(3,3);
      MatrixXd M2 = MatrixXd(3,3);
      MatrixXd M  = MatrixXd(3,3);
      double cp  = cos(p_(0));
      double c2p = cos(2.*p_(0));
      double sp  = sin(p_(0));
      double s2p = sin(2.*p_(0));
      double c11  = p_(1)*p_(1)*Rp_(0,0) - p_(2)*p_(2)*Rp_(1,1);
      double c22  = 2.*p_(1)*Rp_(0,0);
      double c33  = 2.*p_(2)*Rp_(1,1);
  
      M1 <<   -s2p   ,   cp*cp  ,   sp*sp   ,
               c2p   ,    s2p   ,   -s2p    ,
               s2p   ,   sp*sp  ,   cp*cp   ;
      M2 <<    c11   ,     0    ,     0     ,
                0    ,    c22   ,     0     ,
                0    ,     0    ,    c33    ;
            
      M = M1 * M2;
  
      // measurement size = n_z
      int n_z = 2;

      //measurement covariance matrix S
      MatrixXd S = MatrixXd(n_z,n_z);
      S.fill(0.);
      
      //create matrix for sigma points in measurement space
      MatrixXd Zsig = MatrixXd(n_z, 2 * n_augx_ + 1);
      Zsig.fill(0.);
      
      //transform sigma points into measurement space
      for (unsigned i = 0; i < 2*n_augx_+1; i++){
        Zsig(0,i) = Xsig_pred_(0,i);
        Zsig(1,i) = Xsig_pred_(1,i);
      }
      //mean predicted measurement
      VectorXd z_pred = VectorXd(n_z);
      z_pred.fill(0.);

      //calculate mean predicted measurement
      for(int i = 0; i < Zsig.cols(); i++){
        z_pred += weightsx_(i) * Zsig.col(i);
      }
  
      // calculate measurement covariance
      for(int i = 0; i < Zsig.cols(); i++){
        VectorXd z_diff = VectorXd(n_z);
        z_diff = Zsig.col(i) - z_pred;
        S += weightsx_(i) * z_diff * z_diff.transpose();
      }
      S += Ss*Rp_*Ss.transpose() + Rx_;
  
      //measurement
      VectorXd z = VectorXd(n_z);
      z << meas.posX[i] ,
           meas.posY[i] ;
  
      //create matrix for cross correlation Tc
      MatrixXd Tc = MatrixXd(n_x_, n_z);
  
      Tc.fill(0.);
      //calculate cross correlation matrix
      for(int i = 0; i < Zsig.cols(); i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        Tc += weightsx_(i) * x_diff * z_diff.transpose();
      }
  
      //calculate Kalman gain K;
      MatrixXd K = MatrixXd(n_x_, n_z);
      K = Tc * S.inverse();
      
      //update state mean and covariance matrix
      VectorXd z_diff = z - z_pred;
      
      x_  = x_ + K * z_diff;
      Px_ = Px_ - K * S * K.transpose();
      Px_ = (Px_ + Px_.transpose())/2; // forces symmetry

      nisx_ += z_diff.transpose() * S.inverse() * z_diff;
  
      // NOW, THE SHAPE AND ORIENTATION
      MatrixXd T = MatrixXd(3,4);
      T.fill(0.);
      T(0,0) = 1;
      T(1,1) = 1;
      T(2,3) = 1;
      
      VectorXd kron = VectorXd(4);
      kron << z_diff(0) * z_diff(0) ,
              z_diff(0) * z_diff(1) ,
              z_diff(1) * z_diff(0) ,
              z_diff(1) * z_diff(1) ;
      VectorXd Y = VectorXd(3);
      Y = T*kron;
      
      VectorXd E_Y = VectorXd(3);
      E_Y << S(0,0) ,
             S(0,1) ,
             S(1,1) ;
      
      MatrixXd C_YY = MatrixXd(3,3);
      C_YY <<          3 * E_Y(0)*E_Y(0)       ,           3 * E_Y(0)*E_Y(1)     ,  E_Y(0)*E_Y(2) + 2*E_Y(1)*E_Y(1) ,
                       3 * E_Y(0)*E_Y(1)       , E_Y(0)*E_Y(2) + 2*E_Y(1)*E_Y(1) ,            3*E_Y(2)*E_Y(1)       ,
               E_Y(0)*E_Y(2) + 2*E_Y(1)*E_Y(1) ,           3 * E_Y(2)*E_Y(1)     ,            3*E_Y(2)*E_Y(2)       ;
               
      // cross-covariance
      MatrixXd C_pY = MatrixXd(3,3);
      C_pY = Pp_ * M.transpose();
      
      // shape update
      p_  = p_  + C_pY * C_YY.inverse() * (Y-E_Y);
      Pp_ = Pp_ - C_pY * C_YY.inverse() * C_pY;
        Pp_ = (Pp_ + Pp_.transpose()) / 2; // forces symmetry
    }
  }
}

void UKF::NormalizeAngle(double &angle){
  while (angle> M_PI) angle-=2.*M_PI;
  while (angle<-M_PI) angle+=2.*M_PI;
}
