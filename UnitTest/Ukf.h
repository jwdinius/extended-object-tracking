/*
  ==============================================================================

    Ukf.h
    Created: 24 Feb 2018 8:14:38am
    Author:  Joseph Dinius

  ==============================================================================
*/

#ifndef UKF_H
#define UKF_H

#pragma once

#include "../Source/Constants.h"
#include "../Source/Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct SensorUdpTelemetry {
    double timestamp;
    double posX[MAX_DETS];
    double posY[MAX_DETS];
};


class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* state vectors: [] in SI units and rad
  VectorXd x_;
  VectorXd p_;

  ///* state covariance matrices
  MatrixXd Px_;
  MatrixXd Pp_;
    
  ///* measurement covariance matrices
  MatrixXd Rx_;
  MatrixXd Rp_;
    
  ///* process covariance matrices
  MatrixXd Qx_;
  MatrixXd Qp_;
    
  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;
  MatrixXd Psig_pred_;

  ///* time when the state is true, in sec
  double time_;

  ///* Weights of sigma points
  VectorXd weightsx_;
  VectorXd weightsp_;
    

  ///* State dimensions
  int n_x_;
  int n_p_;

  ///* Augmented state dimensions
  int n_augx_;
  int n_augp_;

  ///* Sigma point spreading parameters
  double lambdax_;
  double lambdap_;

  ///* normalized innovation squared
  double nisx_;
  double nisp_;
  
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(SensorUdpTelemetry meas);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */

  void Prediction(double delta_t);

  void NormalizeAngle(double &angle);
  
  /**
   * Updates the state and covariance matrix
   * @param Zsig matrix with sigma points in measurement space
   * @param R measurement covariance matrix
   * @param meas_package The measurement at k+1
   */
  void Update(SensorUdpTelemetry meas);
    
  VectorXd getState();
  
  bool isInitialized() {return is_initialized_;}

};
#endif //UKF_H
