package robot.commands.groups;

import robot.Robot;
import robot.commands.drive.*;
import robot.commands.drive.FollowObject.Object;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class DepositToBay extends CommandGroup {
        
    LimeLight limelight = Robot.limelight;

    public DepositToBay() {

        // get necessary data to drive to perpendicular line 4 feet from bay
        double angleToDriveLine = limelight.solvePerpendicular(4)[0];
        double driveLineLength = limelight.solvePerpendicular(4)[1];
        double angleToPerp = limelight.solvePerpendicular(4)[2];

        // drive to perpendicular using values
        addSequential(new TurnToAngle(angleToDriveLine, 3.0));
        addSequential(new DriveToDistance(driveLineLength, 0.5));
        addSequential(new TurnToAngle(angleToPerp, 3.0));

        // now use vision and line followers to line up
        addSequential(new FollowObject(Object.BAY_CLOSE));
        addSequential(new FollowLine());

    }

}
