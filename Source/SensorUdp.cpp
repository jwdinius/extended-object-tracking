/*
  ==============================================================================

    SensorUdp.cpp
    Created: 17 Nov 2017 12:54:58pm
    Author:  Joseph Dinius

  ==============================================================================
*/

#include "SensorUdp.h"
#include <cmath>
#include <iostream>
#include <limits>

#include "Utils.h"
//#include <random>

void SensorUdp::generateData() {

    int   poisson = distributionP_(generator_);
    while (poisson == 0) poisson = distributionP_(generator_);
    
    //std::cout << poisson << std::endl;
	// DO UPDATE HERE
	// put critical section here
	// truth
	reset();
	tm_.timestamp = Time::highResolutionTicksToSeconds(ptime_->getHighResolutionTicks() - starttime_);
    //DBG(tm_.timestamp);
    //double orient;
    if (tm_.timestamp <= 26.) {
        orient_ = -M_PI/4.;
    }
    else if (tm_.timestamp <= 36. && tm_.timestamp > 26.) {
        orient_ = -M_PI/4 + M_PI/40.*(tm_.timestamp-26.);
    }
    else if (tm_.timestamp <= 57.) {
        orient_ = 0.;
    }
    else if (tm_.timestamp > 57. && tm_.timestamp <= 67.) {
        orient_ = M_PI/20.*(tm_.timestamp - 57.);
    }
    else if (tm_.timestamp <= 83.) {
        orient_ = M_PI/2.;
    }
    else if (tm_.timestamp > 83. && tm_.timestamp <= 93.) {
        orient_ = M_PI/2. + M_PI/20.*(tm_.timestamp - 83.);
    }
    else {
        orient_ = M_PI;
    }
    // from Extended Object Code directly (factor of 10 added to decrease runtime)
    double vx = 5000./36. * cos(orient_);
    double vy = 5000./36. * sin(orient_);
    
    centerX_ += vx * double(MSECS_UDP) / 1000.;
    centerY_ += vy * double(MSECS_UDP) / 1000.;
    
    // record the unperturbed center
    pose_.x     = centerX_;
    pose_.y     = centerY_;
    pose_.theta = orient_;
    //std::cout << pose_.x << ", " << pose_.y << ", " << pose_.theta << std::endl;
    
    Eigen::Vector2d h;
    
    Eigen::Vector2d z, z1;
    Eigen::Matrix2d R;
    Eigen::Vector2d mean;
    
    mean.setZero();
    
    R << cos(orient_) , -sin(orient_) ,
         sin(orient_) ,  cos(orient_) ;
    
    // get measurements (# follows Poisson distribution)
    for (int p = 0; p < poisson; p++){
        h << -1. + 2. * distributionN_(generator_) ,
             -1. + 2. * distributionN_(generator_) ;
        while (h.norm() > 1.) {
            h << -1. + 2. * distributionN_(generator_) ,
                 -1. + 2. * distributionN_(generator_) ;
        }
        z1 << distributionN_(generator_) ,
              distributionN_(generator_) ;
        // total perturbation due to noise
        z = R * size_ * h + mvnrnd(mean, Cv_, z1);
        tm_.posX[p] = centerX_ + z[0];
        tm_.posY[p] = centerY_ + z[1];
    }
    
    /*
    tm_.posX[poisson] = centerX_;
    tm_.posY[poisson] = centerY_;
    */
	//std::cout << tm_.timestamp << ", " << orient_ << ", " << tm_.posX[0] << ", " << tm_.posY[0] << std::endl;
    // estimate
		    
    // end critical section
    
	return;
}

void SensorUdp::reset() {
    for (int i = 0; i < MAX_DETS; i++) {
        tm_.posX[i] = std::numeric_limits<double>::infinity();
        tm_.posY[i] = std::numeric_limits<double>::infinity();
    }
    pose_.x     = 0.;
    pose_.y     = 0.;
    pose_.theta = 0.;
}

void SensorUdp::changeListenerCallback(ChangeBroadcaster *) {
	generateData();
    
    sendChangeMessage();
    pTT_->getNewData();
	return;
}

SensorUdpTelemetry SensorUdp::getTelemetry() {
    SensorUdpTelemetry out;
    out.timestamp = tm_.timestamp;
    for (int i = 0; i < MAX_DETS; i++){
        out.posX[i] = tm_.posX[i];
        out.posY[i] = tm_.posY[i];
    }
    return out;
}

ObjectPose SensorUdp::getPose() {
    ObjectPose out;
    out.x     = pose_.x;
    out.y     = pose_.y;
    out.theta = pose_.theta;
    return out;
}
