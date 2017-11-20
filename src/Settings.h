//
// Created by Stanislav Olekhnovich on 03/11/2017.
//

#ifndef PATH_PLANNING_SETTINGS_H
#define PATH_PLANNING_SETTINGS_H

inline double MphToMs(double mph) { return mph * 0.447; }

const double MaxSpeed = 50.0;
const double OptimalSpeed = MaxSpeed * 0.98;

const double SpeedChangeDelta = 0.224;
const double SimulatorRunloopPeriod = 0.02;

const double MaxSpeedMetersPerSec = MphToMs(MaxSpeed);

const int NumberOfLines = 3;
const double StartingTargetSpeed = 4.0;

#endif //PATH_PLANNING_SETTINGS_H
