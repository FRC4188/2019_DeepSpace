package robot.utils;

import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Pathfinder;

/** Contains paths to be followed. */
public class Paths {

    public static Waypoint[] testPath  = new Waypoint[] {
        new Waypoint(0, 0, 0),
        new Waypoint(4, 0, 0),
        new Waypoint(7, 5, Pathfinder.d2r(45))
    };

}
