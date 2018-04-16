/*
  ==============================================================================

    TimerThread.cpp
    Created: 19 Feb 2018 10:57:39am
    Author:  Joseph Dinius

  ==============================================================================
*/
#include "TimerThread.h"

/*
 * TimerThread
 * simple sleeper thread to enforce rate keeping of sensor callback
*/
void TimerThread::run() {
    while (!threadShouldExit()) {
#ifdef WIN32
		sleep(frameTime_);
#else
		usleep(frameTime_*1000);
#endif
        if (!isThreadDone_) {
			isThreadDone_ = true;
			sendChangeMessage();
		}
	}

	return;
}

TimerThread::~TimerThread() {

}
