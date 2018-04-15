# extended-object-tracking
Source code for JUCE application that implements a multi-threaded extended object tracker and simulation.

![Imgur](https://i.imgur.com/TPGR8lg.gif)

Ground truth of the elliptical object is shown in white, with the measured discrete returns shown as square blocks in and around the object.  The estimated extended object track is shown in <span style="color:green">green</span>.

## Notes
- Code is written in C++ and uses the [JUCE API](https://juce.com/).  The processed measurements are created at a fixed rate, 1Hz, to model fixed rate processing of most sensors, e.g. radar and lidar.
- Algorithm comes from [this paper](https://arxiv.org/pdf/1604.00219.pdf)
