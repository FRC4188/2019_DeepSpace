package robot.commands.groups;

import robot.Robot;
import robot.commands.drive.*;
import robot.commands.drive.FollowObject.Object;
import robot.commands.drive.FollowPath.Path;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class DepositToBay extends CommandGroup {

    LimeLight limelight = Robot.limelight;
    double firstTurn, driveLength, targetAngle;

    public DepositToBay() {

        // drive to perpendicular
        addSequential(new FollowPath(Path.TO_PERPENDICULAR, false));

        // now use vision and line followers to line up
        addSequential(new FollowObject(Object.BAY_HIGH));
        addSequential(new FollowLine());

    }

}
