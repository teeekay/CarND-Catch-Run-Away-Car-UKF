## **Unscented Kalman Filter Project**

---

<img src="https://github.com/teeekay/CarND-Catch-Run-Away-Car-UKF/blob/master/Output/High_STDvals_Runawaycatch.png?raw=true"  width=800>

<i><u>Figure 1: Runaway Car caught even with High Noise on Sensors</u></i>

---






### Writeup by Tony Knight - 2017/08/14

I copied the code used in the [UKF project](https://github.com/teeekay/CarND-Unscented-Kalman-Filter-Project) into [ukf.cpp](https://github.com/teeekay/CarND-Catch-Run-Away-Car-UKF/blob/master/src/ukf.cpp) and [ukf.h](https://github.com/teeekay/CarND-Catch-Run-Away-Car-UKF/blob/master/src/ukf.h).  I then added a new function Predictor() in ukf.cpp which used the code from Prediction(), to predict the location of the car in the future but without overwriting P_ or x_.  This was called in a loop from [main.cpp](https://github.com/teeekay/CarND-Catch-Run-Away-Car-UKF/blob/master/src/main.cpp) to project the location of the car in the future, and find the future location of the runaway car that the hunter car could reach before the runaway car.

This strategy was generally very successful, and the hunter was generally able to capture the car in about 4-4.3 seconds.  I was able to increase the noise on the sensors significantly without adjusting the parameters in ukf and still get the hunter to go straight to the proper location to capture the runaway car.   

|variable | run1 | run2 | run3 |run4 |run5 |
|---------|------|------|------|------|------|
|std_laspx |0.15| 0.25 | 0.4 |0.5|1.0|
|std_laspy |0.15| 0.25 | 0.4 |0.5|1.0|
|std_radr  |0.3 | 0.5 |0.7 |1.0 |1.0|
|std_radphi |0.03| 0.05| 0.07 |0.1|0.1|
|std_radrd |0.3| 0.5 |0.7 |1.0|1.0|
|time to catch (s) |4.3 | 4.10 | 3.96 | 4.44| 14.44*|

* the kalman filter model blew up on run5 two times out of three, but on the third try, the runaway car was caught.  This shows the power of the unscented kalman filter to be able to cut through the noise of noisy sensors