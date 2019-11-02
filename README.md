# FTC #12861 RoboHeroes

<img src= "https://github.com/Scorprion/RoboHeroes-12861/blob/master/FTC%20RoboHeroes%20-%20%20logo%20final_ext.jpg" height="200px" width="200px">

This repository contains the code for FTC Team #12861 RoboHeroes. Our code is organized into folders that are for each season/year. 

## Current Todo:
- Focus on implementing some basic control theory for better auto programs and control
- Make teleop more and more intuitive and smooth
## Goals/Missions
#### Autonomous
~~1. Move forward and turn parallel to stones ~~
2. Move back/foward (slowly) to read the skystones w/ Vuforia
~~3. Once detected, grab the skystone ~~ (Technically)
4. Move forward to detect the color (or Vurforia/distance sensor possibly) and drop the skystone in the building zone
5. Move back and detect the second skystone in the same manner
6. Drop it too


#### TeleOp
> ~~Make a TeleOp~~


**Not Priorities**
> Display Vuforia to the driver station phone screen


## Further Design Details
1. Encoder forward + PI(D) turn sideways
2. Time/Encoder movement + PI(D) straight + Vurofia scan. Once found, PI(D) to align straight with skystone
3. Turn<sup>1</sup> and grab skystone
4. PI(D) straight + Time/Encoder movement. Possibly implement a range using the encoder distance from the saved counts (from where the Vuforia was found and centered) and run to the color sensor detected. Once the color is detected, due to the inaccuracy of the sensor, use the saved counts and current counts to find the total distance (in counts) traveled. Using this distance, using the previously found values, the location of the skystone can be accuratly placed. 
5. Time/Encoder movement + PI(D) straight + Vurofia scan
6. Release clamping machanism

1 The grab mechanism may not require turning

## Design Process
We are currently waiting for the build team to finalize the intake mechanism so we can begin testing and refining programs for it.
