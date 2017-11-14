# Udacity SDC Term 3 Path Planning Project

## First steps, basic planners 

1. I took project sample code from the github and refactored it into something
cleaner, more pleasant to work with and more extendable, as from the start I planned to
implement several planners, ranging from the simplest one to the final one with state machine and cost functions.

2. I've decided to implement simplest planner possible, which will just keep lane and nothing else.
I did it in `KeepLanePathPlanner` class.

3. Then, I've implemented spline based planner, which was shown in the walkthrough video. 
It could keep lane in a smooth way and handle lane change to the left, not taking in consideration other cars.
That kind of planner was implemented as `SimpleSplineBasedPlanner` (it can keep lane, accelerate/deaccelerate, 
if it detects other car closely ahead) and `SplineBasedLeftChnageCapablePlanner` (which can also make lane change to the left).
I've also placed my work at that stage on github as https://github.com/fspirit/path-planning-starter.

## State machine skeleton

I implemented a skeleton for state machine based planner (`FSMPlanner`).
FSM consisted of just one state `KeepLaneState`, which used previosly implemented
`SimpleSplineBasedPlanner` to keep lane and watch for a car ahead.

## Proper lane change trajectory

Now it was a time to implement proper lane changes.
First I decided to implement just a change to the left, debug it, and then 
implement a change to the right and extract common code to the base class.
I quickly discovered that SplineBasedLeftChangeCapablePlaner is breaking
acceleration limit, so I decided to implement new trajectory generator for lane changes
specifically, which will generate path with D values changing smoothly from
current lane's center to target lane's center. This resulted in 
ChangeLanePathGenerator, with descendants LeftChangeLanePathGenerator & RightChangeLanePathGenerator.
I also extracted common code to SplinePathGenerator and moved path generation for
just keeping lane into KeepLanePathGenerator.

## Cost functions implementation and tuning

After I was able to change lane smoothly I needed to implement proper cost 
functions which would tell when to do that.
I've implemented 4 cost functions:

   1. Cost function which motivates to drive with optimal speed. 
    (That  function was shown in the lectures). See `KeepMaxSpeed`
    
   2. Cost function which penalises changing lane on low speeds and 
    changing lanes in general, as if previous func is giving approx same
    values for keeping and changing lanes, its usually safer to stay where you are.
    See `ChangeLaneOnProperSpeed`.
    
   3. To motivate lane changes in proper time, I implemented cost function
    which evaluates speed of closest ahead car on target lane (for keep lane
    path target lane is the same as current). For instance it will give lower cost
    to lane changing path, if closest car on the target line is driving
    faster (or there is no car ahead on the target lane) then the car ahead on the current lane.
    See `SelectFastestLane`.
    
   4. Collision detection cost function. It iterates through candidate path's points,
   and tries to predict other cars' trajectories and gives max cost, if proposed 
   candidate trajectory comes to close to other cars' trajectories at any time step.
   See `AvoidCollision`.

After tuning parameters for those cost functions, I was able to make car drive in a proper way.
 

