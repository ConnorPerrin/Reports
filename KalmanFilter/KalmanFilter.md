# Kalman Filter

## Introduction

What is the `Kalman Filter`?
Simply put, the Kalman filter is a algorithm that quickly estimates the `true value` of the object being measured when the measurement values contain `unpredicted or random errors`.

What is meant by `true value`? This can be a range of values depending on what you are measuring. For example, if we are measuring the temperature of a room, we can safely save that over the period of a few second the temperature is unlikely to change. However, depending on the accuracy of our thermometer, we could record large swings in our data. In this case, the `true value` is the actual temperature of the room.

What is meant by `unpredicted or random errors`?
Like the example just given, in order for us to know the temperate of the room, we have to use some form of sensing device to record the temperature. However, many devices will come with some degree of error (often more expensive devices will come with lower errors). Whilst this accounts for `random errors`, sometimes devices can start to fail unbenounced to the user, these are the `random errors`.

`Michel van Biezen` has a very nice definition to the [Kalman Filter](https://www.youtube.com/watch?v=CaCcOwJPytQ).

> It is an iterative Mathemtical process that uses a set of equation and consectuive data inputs to quickly estimate the true value (position, velocity, etc) of the object being measured, when the measure values containn unpredicted or random error (uncertiany or variations)


## Some examples:

Before diving into how the Kalman Filter actually works, I think it best to have some visual examples of where the Kalman filter is actually used.

### Example one - Measuring the temperature of a room
I know I've given this example already but let me go into a bit more detail.
Imagine we have a thermometer that records the current temperature every second. However, this thermometer has an error of ±0.5 degrees.

To gain a good estimate of the `true` temperature of the room we have two options.

    1. Wait until we have a good number of recorded temperatures and then take an average 
    2. Use a Kalman Filter

Now I know what you're saying, "Why not just use the first option? It's simple right?". Well Kalman Filter have a big advantage... speed. To gain an accurate estimation using averages, you might have to wait for 50+ samples, however Kalman Filters can quickly reach accuracte estimations with far fewer samples.

![](https://github.com/ConnorPerrin/Reports/blob/main/KalmanFilter/images/temperatureExample.png)






# NOTES
---------------------------------------------

Example:
    "We have 50-100 data points that come in one at a time"
    "Could do a distrubition of these values"
    "Average value must be pretty close to the true value"
    "In order to do that we need to have a whole bunch of values already"
    "The Kalman filter does not wait for all the inputs, it very quickly starts to narrow in on the true value"
    "It is an iterative Mathemtical process that uses a set of equation and consectuive data inputs to quickly estimate the true value (position, velocity, etc) of the object being measured, when the measure values containn unpredicted or random error (uncertiany or variations)
    "The data coming in is NOT the true value, it's somewhere around the true value"


## Three main calculation

    1. Calculate the Kalman Gain (also just called the gain)
    2. Calcualte current Estimation
    3. Calculate new error in estimate (error = uncertainty)

## Process

    1. Need the error in the estimate & the error in data (measurements)
    2. Both these feed into the Kalman gain
    3. Gain feeds into the current estimate
       1. This also depends on the previous estimate and the measured value
    4. Then calculate the error in the estimate
       1. We need the 
          1. current estimate
          2. The current gain


## Kalman Gain



## Notes:
    - Used to estimate variables of interest when they cannot be measured directlty (indirectly measured)
    - Combining data from various other sensors
    - 