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

        /*
        firstTurn = limelight.solvePerpendicular(5)[0];
        driveLength = limelight.solvePerpendicular(5)[1];
        targetAngle = limelight.solvePerpendicular(5)[2];

        SmartDashboard.putNumber("first turn", firstTurn);
        SmartDashboard.putNumber("drive length", driveLength);
        SmartDashboard.putNumber("target angle", targetAngle);

        // drive to perpendicular using values
        addSequential(new TurnToAngle(firstTurn, 3.0, Angle.ABSOLUTE));
        addSequential(new DriveToDistance(driveLength, 1.0, Distance.RELATIVE));
        addSequential(new TurnToAngle(targetAngle, 3.0, Angle.ABSOLUTE));
        */

        // now use vision and line followers to line up
        addSequential(new FollowObject(Object.BAY_CLOSE));
        addSequential(new FollowLine());

    }

}
