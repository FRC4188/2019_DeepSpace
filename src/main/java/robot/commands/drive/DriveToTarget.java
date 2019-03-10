package robot.commands.drive;

import robot.OI;
import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/** Drives forward using pilot left joystick while
 *  turning to keep given vision target centered. */
public class DriveToTarget extends Command {

    public enum VisionTarget {
        CARGO, BAY, BAY_3D
    }

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    VisionTarget target;
    final double TURN_kP = 0.01;

    public DriveToTarget(VisionTarget target) {
        requires(Robot.drivetrain);
        requires(Robot.limelight);
        setName("DriveToTarget: " + target.toString());
        this.target = target;
    }

    @Override
    protected void initialize() {

        if(target == VisionTarget.CARGO) {
            limelight.trackCargo();
        } else if(target == VisionTarget.BAY) {
            limelight.trackBay();
        } else if(target == VisionTarget.BAY_3D) {
            limelight.trackBay3D();
        }

    }

    @Override
    protected void execute() {

        // use pilot left joystick for xSpeed
        double xSpeed = Robot.oi.getPilotY(Hand.kLeft);

        // angle p loop
        double angleErr = limelight.getHorizontalAngle();
        double zTurn = TURN_kP * angleErr;

        // command motor output
        drivetrain.arcade(xSpeed, zTurn);
        System.out.println("driving to target");

    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }

}
