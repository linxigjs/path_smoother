#ifndef PATH_SMOOTHER_CONSTANTS_H
#define PATH_SMOOTHER_CONSTANTS_H

namespace Constants {
    /// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
    static const int max_iterations = 200;

    /// [m] --- The minimum turning radius of the vehicle
    static const float min_turn_radius = 6;

    /// [m] --- The minimum width of a safe road for the vehicle at hand
    static const float minRoadWidth = 2;
}

#endif // PATH_SMOOTHER_CONSTANTS_H

