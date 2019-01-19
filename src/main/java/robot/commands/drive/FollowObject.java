package robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;

public class FollowObject extends Command {

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    public FollowObject() {
        requires(Robot.drivetrain);
        requires(Robot.limelight);
    }

    @Override
    protected void initialize() {
        limelight.trackCargo();
        drivetrain.resetTurnToAngle();
    }

    @Override
    protected void execute() {
        SmartDashboard.putNumber("Cargo angle", limelight.getHorizontalAngle());
        drivetrain.turnToAngle(limelight.getHorizontalAngle(), 1.0);
    }

    @Override
    protected boolean isFinished() {
      return false;
    }

    @Override
    protected void end() {
        drivetrain.resetTurnToAngle();
    }

    @Override
    protected void interrupted() {
    }
}
