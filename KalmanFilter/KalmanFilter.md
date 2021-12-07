# Kalman Filter

## Introduction

What is the `Kalman Filter`?
Simply put, the Kalman filter is an algorithm that quickly estimates the `true value` of the object being measured when the measurement values contain `unpredicted or random errors`.

What is meant by `true value`? This can be a range of values depending on what you are measuring. For example, if we are measuring the temperature of a room, we can safely say that over the period of a few second the temperature is unlikely to change. However, depending on the accuracy of our thermometer, we could record large swings in our data. In this case, the true value is the actual temperature of the room.

What is meant by `unpredicted or random errors`?
Like the example just given, in order for us to know the temperate of the room, we have to use some form of sensing device to record the temperature. However, many devices will come with some degree of error (often more expensive devices will come with lower errors). Whilst this accounts for random errors, sometimes devices can start to fail unbeknownst to the user, these are the random errors.

`Michel van Biezen` has a very nice definition to the [Kalman Filter](https://www.youtube.com/watch?v=CaCcOwJPytQ).

> It is an iterative mathematical process that uses a set of equation and consecutive data inputs to quickly estimate the true value (position, velocity, etc) of the object being measured, when the measure values contain unpredicted or random error (uncertainly or variations)


## Some examples:

Before diving into how the Kalman Filter actually works, I think it best to have some visual examples of where the Kalman filter is actually used.

### Example one - Measuring the temperature of a room

I know I've given this example already but let me go into a bit more detail.
Imagine we have a thermometer that records the current temperature every second. However, this thermometer has an error of ±0.5 degrees.

To gain a good estimate of the `true` temperature of the room we have two options.

    1. Wait until we have a good number of recorded temperatures and then take an average 
    2. Use a Kalman Filter

Now I know what you're saying, "Why not just use the first option? It's simple right?". Well Kalman Filter have a big advantage... speed. To gain an accurate estimation using averages, you might have to wait for 50+ samples, however Kalman Filters can quickly reach accurate estimations with far fewer samples.

![](https://github.com/ConnorPerrin/Reports/blob/main/KalmanFilter/images/temperatureExample.png)

The picture above shows the `true value` (red line), the `recorded values` (black crosses taken every 1 second) and the result of applying the `Kalman Filter` (green line).
As you can see from the picture, our sampled data is seemingly random. However, you'll notice that after just a few samples, the Kalman approximation starts to converge close to the actual value.

### Example two - Approximating the temperature of something we cannot directly measure 

Bit of a confusing title right? Think of it this way. Imagine we have a rocket engine and we want to record the temperature of the main combustion chamber, however combustion chambers can reach into the thousands of degrees (2,500c / 4,500f). Currently there doesn't exist any kind of thermometer that can survive that level of heat. But lets say that we have to record the temperature for scientific reasons, how do we do it? You guessed it, `Kalman Filters`. Kalman filters allow us to approximately predict a values from an `indirect measurements`.

So what is an `indirect measurement` source? In our example, this could be a thermometer placed on the outside of the combustion chamber. For example, take a look at the incredibly accurate picture of a rocket below.

![](https://github.com/ConnorPerrin/Reports/blob/main/KalmanFilter/images/rocketEngineExample.png)

However, there is something else we need. In order to get an accurate prediction of the temperature, we first need an estimation. "How do we estimate the temperature?" I hear you say, for that we need to do some maths. Luckily for us, it is relatively easy to create a model that takes in several parameters and estimates both the internal and external temperature of the combustion engine. With this information, along with the actual recorded external temperature, we can accurately predict the internal temperature.



TODO

![](https://github.com/ConnorPerrin/Reports/blob/main/KalmanFilter/images/RocketTemperatureModelFlow.png)



### Example three - Combining sensors

Car example...


## Limitation of the Kalman Filter

linear


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