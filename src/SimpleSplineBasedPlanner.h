//
// Created by Stanislav Olekhnovich on 17/10/2017.
//

#ifndef PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H
#define PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H

#include "PathPlanner.h"
#include "spline.h"
#include "PathGenerators/SplinePathGenerator.h"

class SimpleSplineBasedPlanner : public PathPlanner
{
public:
    explicit SimpleSplineBasedPlanner(const HighwayMap& map, int startingLane, double targetSpeed):
            PathPlanner(map, startingLane), targetSpeed(targetSpeed) {};
    std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
    double GetTargetSpeed() const { return targetSpeed; }

protected:
    double targetSpeed;
    virtual std::shared_ptr<SplinePathGenerator> CreatePathGenerator(bool otherCarTooCloseAhead);

private:
    bool IsTooCloseToOtherCar(const PathPlannerInput &input) const;

};



#endif //PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H
