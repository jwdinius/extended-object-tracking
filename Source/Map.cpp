/*
  ==============================================================================

    Map.cpp
    Created: 19 Feb 2018 10:57:39am
    Author:  Joseph Dinius

  ==============================================================================
*/
#include "../JuceLibraryCode/JuceHeader.h"
#include "Map.h"

/*
 * paint method
 * plot outputs of sensor and Kalman filters
 * this uses graphical methods from the JUCE API
*/
void Map::paint (Graphics& g)
{
    
    // Clear the background
	g.fillAll (Colours::black);
	
	g.setColour (Colours::black);
    g.drawRect (getLocalBounds(), 1);
    
    // find middle of window
    float x_origin = .5f * (float)getWidth();
    float y_origin = .5f * (float)getHeight();
    float square = 10.f * (float)getWidth()/MAP_WIDTH;
    g.setColour (Colours::white);
    
    // plot discrete measurements
    for (int i = 0; i < MAX_DETS; i++)
    {
        if (tm_.posX[i] < std::numeric_limits<double>::infinity()) {
            float xs = x_origin + .5f * (float)getWidth() / MAP_WIDTH * tm_.posX[i];
            float ys = y_origin - .5f * (float)getHeight() / MAP_HEIGHT * tm_.posY[i];
            
            g.fillRect(xs-square, ys-square, 2.f * square, 2.f * square);
        }
    }
    
    // plot ground truth object
    if (pose_.x < std::numeric_limits<double>::infinity()){
        Path truth;
        double xt, yt, rx, ry;
        xt = x_origin + .5f*(float)pose_.x * (float)getWidth() / MAP_WIDTH;
        yt = y_origin - .5f*(float)pose_.y * (float)getHeight()/ MAP_HEIGHT;
        rx = OBJECT_W * (float)getWidth()/MAP_WIDTH;
        ry = OBJECT_H * (float)getHeight()/MAP_HEIGHT;
        truth.addCentredArc(xt, yt, rx, ry, -pose_.theta, 0.f, 2.f*M_PI, true);
    
        g.strokePath(truth, PathStrokeType(2.0f) );
    }
    
    // plot Kalman filter outputs
    g.setColour (Colours::limegreen);
    if (is_kf_initialized_){
        Path kf;
        double xu, yu, rxu, ryu, au;
        xu = x_origin + .5f*(float)kf_state_[0] * (float)getWidth() / MAP_WIDTH;
        yu = y_origin - .5f*(float)kf_state_[1] * (float)getHeight()/ MAP_HEIGHT;
        rxu = kf_state_[5] * (float)getWidth()/MAP_WIDTH;
        ryu = kf_state_[6] * (float)getHeight()/MAP_HEIGHT;
        au = -kf_state_[4];
        kf.addCentredArc(xu, yu, rxu, ryu, au, 0.f, 2.f*M_PI, true);
        g.strokePath(kf, PathStrokeType(2.0f) );
    }
}

/*
 * setTelemetry(tm)
 * set data for plotting
*/
void Map::setTelemetry(SensorUdpTelemetry tm)
{
    for (int i = 0; i < MAX_DETS; i++)
    {
        tm_.timestamp = tm.timestamp;
        tm_.posX[i]   = tm.posX[i];
        tm_.posY[i]   = tm.posY[i];
    }
    return;
}

/*
 * setPose(pose)
 * ground truth for plotting
*/
void Map::setPose(ObjectPose pose)
{
    pose_.x     = pose.x;
    pose_.y     = pose.y;
    pose_.theta = pose.theta;
    return;
}

/*
 * setKfState(state)
 * set Kalman filter data for plotting
*/
void Map::setKfState(VectorXd state)
{
    kf_state_ = state;
    return;
}

void Map::resized()
{
    // This method is where you should set the bounds of any child
    // components that your component contains..

}


