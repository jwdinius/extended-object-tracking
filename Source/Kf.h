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
  
  //* pseudomeasurement transformation
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
   * ProcessMeasurement(meas)
   */
  void ProcessMeasurement(SensorUdpTelemetry meas);

  /**
   * Prediction() - propagates state/covariance estimates
   */
  void Prediction();

  void NormalizeAngle(double &angle);
  
  /**
   * Update(meas) - Kalman filter update method
   */
  void Update(SensorUdpTelemetry meas);
    
  VectorXd getState();
  
  bool isInitialized() {return is_initialized_;}
  
  void printOutput();

};
#endif //KF_H
