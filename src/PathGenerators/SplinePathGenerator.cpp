//
// Created by Stanislav Olekhnovich on 08/11/2017.
//

#include "SplinePathGenerator.h"

#include <iostream>
#include <algorithm>

#include "../Settings.h"

double deg2rad(double x) { return x * M_PI / 180.0; }

static const int XAxisPlanningHorizon = 30;
static const int MaxNumberOfPointsInPath = 50;

std::vector<CartesianPoint> SplinePathGenerator::GeneratePath(const PathPlannerInput& input)
{
    auto anchorsGenerationResult = GenerateAnchorPoints(input);
    auto anchorsLocal = ConvertPointsToLocalSystem(anchorsGenerationResult.AnchorPoints, anchorsGenerationResult.ReferencePoint);

    auto anchorsBasedSpline = GetSplineFromAnchorPoints(anchorsLocal);
    auto newPathPoints = GenerateNewPointsWithSpline(anchorsBasedSpline, (int)input.Path.size());

    std::vector<CartesianPoint> outputPath = { input.Path };
    for (auto& p: newPathPoints)
        outputPath.push_back(p.ToGlobal(anchorsGenerationResult.ReferencePoint));

    return outputPath;
}

SplinePathGenerator::AnchorPointsGenerationResult SplinePathGenerator::GenerateAnchorPoints(const PathPlannerInput& input) const
{
    CartesianPoint referencePoint = input.LocationCartesian;

    referencePoint.Theta = deg2rad(referencePoint.Theta);

    std::vector<CartesianPoint> anchors;
    if (input.Path.empty() || input.Path.size() == 1)
    {
//        std::cout << "Generating from empty path" << std::endl;
        anchors.push_back({referencePoint.X - cos(referencePoint.Theta),
                           referencePoint.Y - sin(referencePoint.Theta)});
        anchors.push_back(referencePoint);
    }
    else
    {
        referencePoint = input.Path.back();
        auto prevPoint = input.Path[input.Path.size() - 2];

        referencePoint.Theta = atan2(referencePoint.Y - prevPoint.Y, referencePoint.X - prevPoint.X);

        anchors.push_back(prevPoint);
        anchors.push_back(referencePoint);
    }

    AddAnchors(input, anchors);

    return { referencePoint, anchors };
}

std::vector<CartesianPoint> SplinePathGenerator::ConvertPointsToLocalSystem(const std::vector<CartesianPoint> &anchorPointsGlobal,
                                                                                 const CartesianPoint &localReferencePoint) const
{
    std::vector<CartesianPoint> anchorPointsLocal;
    for (auto& p: anchorPointsGlobal)
    {
        anchorPointsLocal.push_back(p.ToLocal(localReferencePoint));
    }
    return anchorPointsLocal;
}

std::vector<CartesianPoint>
SplinePathGenerator::GenerateNewPointsWithSpline(const tk::spline &newPathSpline, int pointsLeftInCurrentPath) const
{
    const double pathEndpointX = 30;
    double pathEndpointY = newPathSpline(pathEndpointX);
    double pathLength = sqrt(pathEndpointX * pathEndpointX + pathEndpointY * pathEndpointY);

    std::vector<CartesianPoint> pathPoints;

    double prevX = 0;
    double numberOfPoints = pathLength / (SimulatorRunloopPeriod * MphToMs(targetSpeed));
    double xAxisStep = XAxisPlanningHorizon / numberOfPoints;
    for (int i = 1; i <= MaxNumberOfPointsInPath - pointsLeftInCurrentPath; i++)
    {
        double x = prevX + xAxisStep;
        double y = newPathSpline(x);

        prevX = x;
        pathPoints.emplace_back(x, y);
    }
    return pathPoints;
}

tk::spline SplinePathGenerator::GetSplineFromAnchorPoints(const std::vector<CartesianPoint> &newPathAnchorPoints) const
{
    std::vector<double> newPathAnchorsX;

    std::vector<double> newPathAnchorsY;
    for (auto& p: newPathAnchorPoints)
    {
        newPathAnchorsX.push_back(p.X);
        newPathAnchorsY.push_back(p.Y);
    }
    tk::spline spline;
    spline.set_points(newPathAnchorsX, newPathAnchorsY);
    return spline;
}
