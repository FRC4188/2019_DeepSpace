package robot.commands.groups;

import robot.utils.*;
import robot.commands.drive.*;
import robot.commands.drive.FollowPath.Path;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class LHab2ToFrontShip extends CommandGroup {

    public LHab2ToFrontShip() {

        addParallel(new Wait(), 1.0);
        addParallel(new FollowPath(Path.L_TO_L_FRONT_SHIP_HAB2, false));

    }

}