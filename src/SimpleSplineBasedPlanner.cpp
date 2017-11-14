//
// Created by Stanislav Olekhnovich on 17/10/2017.
//

#include <vector>
#include <iostream>

#include "SimpleSplineBasedPlanner.h"
#include "Settings.h"
#include "PathGenerators/KeepLanePathGenerator.h"

const double CriticalThresholdInMeters = 30;

std::vector<CartesianPoint> SimpleSplineBasedPlanner::GeneratePath(PathPlannerInput input)
{
    currentLane = input.LocationFrenet.Lane();

    bool otherCarTooCloseAhead = IsTooCloseToOtherCar(input);
    if (otherCarTooCloseAhead)
    {
        targetSpeed -= SpeedChangeDelta;
    }
    else if (targetSpeed < OptimalSpeed)
    {
        targetSpeed += SpeedChangeDelta;
    }

    std::shared_ptr<SplinePathGenerator> pathGenerator = CreatePathGenerator(otherCarTooCloseAhead);

    return pathGenerator->GeneratePath(input);
}

bool SimpleSplineBasedPlanner::IsTooCloseToOtherCar(const PathPlannerInput &input) const
{
    double egoPredictedEndpointS = !input.Path.empty() ? input.PathEndpointFrenet.S : input.LocationFrenet.S;

    for (auto& otherCar : input.OtherCars)
    {
        if (otherCar.IsInLane(currentLane))
        {
            double otherCarPredictedS = otherCar.LocationFrenet.S +
                                        (input.Path.size() * SimulatorRunloopPeriod * MphToMs(otherCar.Speed2DMagnitude()) );
            if (otherCarPredictedS > egoPredictedEndpointS &&
                    (otherCarPredictedS - egoPredictedEndpointS) < CriticalThresholdInMeters)
                return true;
        }
    }
    return false;
}

std::shared_ptr<SplinePathGenerator>
SimpleSplineBasedPlanner::CreatePathGenerator(bool otherCarTooCloseAhead) {
    return std::make_shared<KeepLanePathGenerator>(targetSpeed, map);
}



