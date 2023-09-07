# 3128-common
<!-- [![Build CI](https://github.com/MoSadie/RobotLib/actions/workflows/gradleCI.yml/badge.svg?branch=actions)](https://github.com/MoSadie/RobotLib/actions/workflows/gradleCI.yml)
[![](https://jitpack.io/v/ORF-4450/robotlib.svg)](https://jitpack.io/#ORF-4450/robotlib) -->

FRC Team 3128 Robot Control program library classes.

These are the library classes created by the Olympia Robotics Federation (FRC Team 3128).
This library is delivered via jar file for use in our robot control programs.

## How to download

### GradleRIO (2019 full release and later)

Add the file 3128-common.json from this project to the vendordeps directory of your robot project. You can use the
file url from the list above to add to VSCode with the WPILib: Manage Vendor Libraries command. Check the 3128-common version inside that file and set the version you wish to pull from Jitpack. Normally the version in 3128-common.json 
will point to the current release. Recompile while connected to the internet.

Import of Javadocs, source attachment, and jars will be done automatically when building the robot project.

### WARNING ###

This library no longer contains its dependencies. You have to import the dependent libraries in the robot project
consuming this library. As of 3.0 these libraries are needed: Navx, CTRE_Phoenix. You can copy the json files for
these libraries from 3128-common vendordeps folder to your robot project vendordeps folder. Don't forget to do a 
Gradle refresh after changing any vendordeps file. As of 3.4.0 you also need REVColorSensorV3.

### NOTICE

v2.x is not compatible with 2019 (full release) or later FRC robotics platform. Only use this library for pre-2019 projects that will run on a RoboRIO with a pre-2019 image.

### NOTICE TO DEVELOPERS
Read the documentation in build.gradle for the procedure to do development on this
library and then generate a release on Github and Jitpack.
