package robot.commands.groups;

import robot.utils.*;
import robot.commands.drive.*;
import robot.commands.drive.FollowPath.Path;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class RHab2ToFrontShip extends CommandGroup {

    public RHab2ToFrontShip() {

        addParallel(new Wait(), 1.0);
        addParallel(new FollowPath(Path.R_TO_R_FRONT_SHIP_HAB2, false));

    }

}