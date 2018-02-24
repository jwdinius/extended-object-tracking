/*
  ==============================================================================

    SensorUdp.h
    Created: 17 Nov 2017 12:54:58pm
    Author:  Joseph Dinius

  ==============================================================================
*/
#ifndef SENSORUDP_H
#define SENSORUDP_H

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include "Constants.h"
#include "TimerThread.h"
#include "Eigen/Dense"

#include <random>
//#include <unistd.h>

struct SensorUdpTelemetry {
    double timestamp;
    double posX[MAX_DETS];
    double posY[MAX_DETS];
};

struct ObjectPose {
    double x, y, theta;
};

class SensorUdp : public ChangeBroadcaster, // broadcasts to World
                  public ChangeListener     // listens to TimerThread
{
public:
	SensorUdp() {
	    pTT_ = new TimerThread(MSECS_UDP);
	    pTT_->addChangeListener(this);
	    pTT_->startThread();
	    ptime_ = new Time();
	    starttime_ = ptime_->getHighResolutionTicks();
	    isThreadDone_ = false;
	    centerX_ = -4000.;
	    centerY_ = 0.;
        std::poisson_distribution<int> distributionP(P_LAMBDA);
        std::normal_distribution<double> distributionN(0.,1.);
        distributionP_ = distributionP;
        distributionN_ = distributionN;
        Cv_ << 2000. ,  0.  ,
                 0.  , 80.  ;
        size_ << OBJECT_W ,    0.     ,
                    0.    ,  OBJECT_H ;
	}
	~SensorUdp() {
	    if (pTT_ != nullptr) {
	        pTT_->stopThread(2000);
	        pTT_->removeChangeListener(this);
		    delete pTT_;
	    }
	};
	void generateData();
	void changeListenerCallback(ChangeBroadcaster *);
	SensorUdpTelemetry getTelemetry();
	ObjectPose getPose();
	void getNewData() { isThreadDone_ = true; }
	void reset();
			
private:
    //int time_;
	SensorUdpTelemetry tm_;
	ObjectPose pose_;
	
	TimerThread *pTT_;
	Time *ptime_;
	int64 starttime_;
	bool isThreadDone_;
	double centerX_;
	double centerY_;
	double orient_;
	std::default_random_engine generator_;
	std::poisson_distribution<int> distributionP_;
	std::normal_distribution<double> distributionN_;
	Eigen::Matrix2d Cv_;
	Eigen::Matrix2d size_;
};

#endif
