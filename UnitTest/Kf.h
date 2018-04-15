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

#define MAP_WIDTH           8000.0f
#define MAP_HEIGHT          8000.0f

#define MSECS_UDP           1000
#define MAX_DETS            64
#define P_LAMBDA            5

#define OBJECT_W            170.0f
#define OBJECT_H             40.0f

// initial covariance terms
#define INIT_VAR_LENGTH          400.0f
#define INIT_VAR_ALPHA            0.02f
#define INIT_VAR_POS             900.0f
#define INIT_VAR_VEL              16.0f

// process covariance terms
#define VAR_LENGTH          0.5f
#define VAR_ALPHA           0.04f
#define VAR_POS             100.0f
#define VAR_VEL             1.0f

// measurement covariance terms
#define VAR_H               0.25f
#define VAR_V1              2000.0f
#define VAR_V2              80.0f

//#include "SensorUdp.h"

#include "../Source/Eigen/Dense"
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

struct SensorUdpTelemetry {
    double timestamp;
    double posX[MAX_DETS];
    double posY[MAX_DETS];
};

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
