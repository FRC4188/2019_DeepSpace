package robot.commands.groups;

import robot.Robot;
import robot.commands.drive.*;
import robot.commands.drive.DriveToDistance.Distance;
import robot.commands.drive.FollowObject.Object;
import robot.commands.drive.TurnToAngle.Angle;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class DepositToBay extends CommandGroup {
        
    LimeLight limelight = Robot.limelight;
    double firstTurn, driveLength, targetAngle;

    public DepositToBay() {

        // drive to perpendicular
        addSequential(new TurnToAngle(Angle.PERP_FIRST));
        addSequential(new DriveToDistance(Distance.PERP_LENGTH));
        addSequential(new TurnToAngle(Angle.TARGET));

        // now use vision and line followers to line up
        addSequential(new FollowObject(Object.BAY_HIGH));
        addSequential(new FollowLine());

    }

}
