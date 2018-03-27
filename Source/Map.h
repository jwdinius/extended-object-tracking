#ifndef MAP_H
#define MAP_H

#include "../JuceLibraryCode/JuceHeader.h"
#include "SensorUdp.h"
#include "Eigen/Dense"
#include <fstream>

using Eigen::VectorXd;
#include "Constants.h"
#include <limits>


class Map    : public Component
{
public:
    Map() {
        for (int i = 0; i < MAX_DETS; i++){
            tm_.posX[i] = std::numeric_limits<double>::infinity();
            tm_.posY[i] = std::numeric_limits<double>::infinity();
        }
        pose_.x = std::numeric_limits<double>::infinity();
        pose_.y = std::numeric_limits<double>::infinity();
        is_ukf_initialized_ = false;
    };
    ~Map() {};
    
    void paint (Graphics&);
    void resized();
    void setTelemetry(SensorUdpTelemetry tm);
    void setPose(ObjectPose pose);
    void setUkfState(VectorXd state);
    void setUkfInitialized(bool init_flag) { is_ukf_initialized_ = init_flag ;}

private:
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Map)
    
    SensorUdpTelemetry tm_;
    ObjectPose pose_;
    VectorXd ukf_state_;
    bool is_ukf_initialized_;
	
};

#endif
