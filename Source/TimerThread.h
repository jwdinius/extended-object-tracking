#ifndef TIMERTHREAD_H
#define TIMERTHREAD_H

#include "../JuceLibraryCode/JuceHeader.h"
#include "Constants.h"
//#include <unistd.h>

class TimerThread : public Thread, 
                    public ChangeBroadcaster {

public:
	TimerThread(int frameTime) : Thread ("TimerThread") {
	    frameTime_    = frameTime;
		isThreadDone_ = false;
	}
	~TimerThread();
	void run();
	void getNewData()	{ isThreadDone_ = false; }
	bool isThreadDone()	{ return isThreadDone_; }

private:
    int  frameTime_;
	bool isThreadDone_;

};

#endif
