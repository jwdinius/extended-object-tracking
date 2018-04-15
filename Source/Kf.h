/*
  ==============================================================================

    Kf.h
    Created: 24 Feb 2018 8:14:38am
    Author:  Joseph Dinius

  ==============================================================================
*/

#ifndef KF_H
#define KF_H

#pragma once


#include "SensorUdp.h"

#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class KF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* state vectors: [] in SI units and rad
  Vector4d r_;
  Vector3d p_;

  ///* state covariance matrices
  Matrix4d Cr_;
  Matrix3d Cp_;
    
  ///* measurement covariance matrices
  Matrix2d Ch_;
  Matrix2d Cv_;
    
  ///* process covariance matrices
  Matrix4d Cwr_;
  Matrix3d Cwp_;
    
  ///* time delta
  double dt_;
  
  ///* state transition matrices
  Matrix4d Ar_;
  Matrix3d Ap_;
  
  //* measurement jacobian
  MatrixXd H_;

  /**
   * Constructor
   */
  KF();

  /**
   * Destructor
   */
  virtual ~KF();

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

  void Prediction();

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
  
  void printOutput();

};
#endif //KF_H
