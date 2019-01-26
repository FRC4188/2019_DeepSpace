package robot.commands.groups;

import robot.Robot;
import robot.commands.drive.*;
import robot.commands.drive.DriveToDistance.Distance;
import robot.commands.drive.FollowObject.Object;
import robot.commands.drive.TurnToAngle.Angle;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DepositToBay extends CommandGroup {
        
    LimeLight limelight = Robot.limelight;
    double firstTurn, driveLength, targetAngle;

    public DepositToBay() {

        // now use vision and line followers to line up
        addSequential(new FollowObject(Object.BAY_HIGH));
        addSequential(new FollowLine());

    }

}
