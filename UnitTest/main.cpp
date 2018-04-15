#include "Kf.h"
#include <cmath>
#include <limits>

int main() {
	KF *kf = new KF();
	kf->Prediction();
	//! begin unit test setup
	SensorUdpTelemetry tele;
	for (int i = 0; i < MAX_DETS; i++) {
        tele.posX[i] = std::numeric_limits<double>::infinity();
        tele.posY[i] = std::numeric_limits<double>::infinity();
    }
    tele.posX[0] = -117.12838068357;
    tele.posY[0] =  19.9771280201263;
    tele.posX[1] = -34.9920267483934;
    tele.posY[1] =  29.5292236176723;
    //! end unit test setup
	kf->Update(tele);

	delete kf;
	return 0;
}