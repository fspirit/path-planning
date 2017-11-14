//
// Created by Stanislav Olekhnovich on 13/10/2017.
//
#include "HighwayMap.h"

#include <thread>
#include <uWS/uWS.h>
#include <fstream>
#include <sstream>

HighwayMap::HighwayMap(const std::string &highwayMapCsvPath)
{
    ReadMapFromCsvFile(highwayMapCsvPath);
}

CartesianPoint HighwayMap::FrenetToCartesian(const FrenetPoint& frenetPoint) const
{
    int previousPointIndex = -1;
    while(frenetPoint.S > mapPointsS[previousPointIndex+1] && (previousPointIndex < (int)(mapPointsS.size()-1) ))
    {
        previousPointIndex++;
    }
    auto previousPoint = mapPoints[previousPointIndex];

    auto nextPointIndex = static_cast<int>((previousPointIndex + 1) % mapPoints.size());
    auto nextWayPoint = mapPoints[nextPointIndex];

    double heading = atan2((nextWayPoint.Y - previousPoint.Y),(nextWayPoint.X - previousPoint.X));
    // the x,y,s along the segment
    double seg_s = frenetPoint.S - mapPointsS[previousPointIndex];

    double seg_x = previousPoint.X + seg_s*cos(heading);
    double seg_y = previousPoint.Y + seg_s*sin(heading);

    double perp_heading = heading - M_PI_2;

    double x = seg_x + frenetPoint.D * cos(perp_heading);
    double y = seg_y + frenetPoint.D * sin(perp_heading);

    return {x,y};
}

FrenetPoint HighwayMap::CartesianToFrenet(const CartesianPoint& cartesianPoint) const
{
    int nextWayPointIndex = NextWaypoint(cartesianPoint);
    auto prevWayPointIndex = nextWayPointIndex != 0 ? (nextWayPointIndex - 1) : mapPoints.size() - 1;

    auto nextWayPoint = mapPoints[nextWayPointIndex];
    auto prevWayPoint = mapPoints[prevWayPointIndex];

    CartesianPoint nextRelativeToPrev = {nextWayPoint.X - prevWayPoint.X, nextWayPoint.Y - prevWayPoint.Y};
    CartesianPoint currentRelativeToPrev = {cartesianPoint.X - prevWayPoint.X, cartesianPoint.Y - prevWayPoint.Y};

    // find the projection of x onto n
    double projectionNorm = (currentRelativeToPrev.X * nextRelativeToPrev.X + currentRelativeToPrev.Y * nextRelativeToPrev.Y) /
            (nextRelativeToPrev.X * nextRelativeToPrev.X + nextRelativeToPrev.Y * nextRelativeToPrev.Y);
    CartesianPoint currentLocationProjected = {projectionNorm * nextRelativeToPrev.X,
                                               projectionNorm * nextRelativeToPrev.Y};

    double dValue = EuclidDistance(currentRelativeToPrev, currentLocationProjected);

    //see if d value is positive or negative by comparing it to a center point
    CartesianPoint center = { 1000 - prevWayPoint.X, 2000 - prevWayPoint.Y };
    double centerToPos = EuclidDistance(center, currentRelativeToPrev);
    double centerToRef = EuclidDistance(center, currentLocationProjected);

    if(centerToPos <= centerToRef)
    {
        dValue *= -1;
    }

    double sValue = 0;
    for(int i = 0; i < prevWayPointIndex; i++)
    {
        sValue += EuclidDistance(mapPoints[i], mapPoints[i + 1]);
    }
    sValue += EuclidDistance({0,0} , currentLocationProjected);

    return {sValue, dValue};
}

int HighwayMap::NextWaypoint(CartesianPoint currentVehicleLocation) const
{
    int closestWayPointIndex = ClosestWaypoint(currentVehicleLocation);

    auto closestWayPoint = mapPoints[closestWayPointIndex];

    double heading = atan2( (closestWayPoint.Y - currentVehicleLocation.Y),(closestWayPoint.X - currentVehicleLocation.X) );

    double angle = std::abs(currentVehicleLocation.Theta - heading);

    if (angle > M_PI_4)
    {
        closestWayPointIndex++;
    }

    return closestWayPointIndex;
}

int HighwayMap::ClosestWaypoint(CartesianPoint currentVehicleLocation) const
{
    std::vector<double> distances;
    std::transform(mapPoints.begin(),
                   mapPoints.end(),
                   std::back_inserter(distances),
                   [this, &currentVehicleLocation](const CartesianPoint &p){ return EuclidDistance(currentVehicleLocation, p); } );
    auto result = std::min_element(std::begin(distances), std::end(distances));
    return std::distance(std::begin(distances), result);
}

void HighwayMap::ReadMapFromCsvFile(const std::string &highwayMapCsvPath)
{
    std::ifstream in_map_(highwayMapCsvPath.c_str(), std::ifstream::in);

    std::string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        double x, y, s, d_x, d_y;
        iss >> x >> y >> s >> d_x >> d_y;

        mapPoints.emplace_back(x, y);
        mapPointsS.push_back(s);
        mapPointsDX.push_back(d_x);
        mapPointsDY.push_back(d_y);
    }
}


