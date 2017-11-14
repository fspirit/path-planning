//
// Created by Stanislav Olekhnovich on 25/10/2017.
//

#ifndef PATH_PLANNING_BEHAVIORSTATE_H
#define PATH_PLANNING_BEHAVIORSTATE_H

#include <memory>

#include "../../CartesianPoint.h"
#include "../../PathPlannerInput.h"
#include "../../HighwayMap.h"

class BehaviorState
{
public:
    BehaviorState(const HighwayMap& map, int currentLane, double targetSpeed) :
            map(map), currentLane(currentLane), targetSpeed(targetSpeed), targetLane(currentLane) {}
    virtual std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) = 0;
    virtual std::vector<std::shared_ptr<BehaviorState>> GetNextStates(int currentLane) = 0;
    virtual std::string GetName() const = 0;
    int GetTargetLane() const {  return targetLane; }
    double GetTargetSpeed() const { return targetSpeed; }
    virtual ~BehaviorState() {};
protected:
    const HighwayMap& map;
    int currentLane;
    int targetLane;
    double targetSpeed;
};


#endif //PATH_PLANNING_BEHAVIORSTATE_H
