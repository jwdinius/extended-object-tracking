#include "TimerThread.h"

void TimerThread::run() {
    int64 currentTime, pastTime, deltaTime;
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
