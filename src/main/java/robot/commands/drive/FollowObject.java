package robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import robot.utils.CSPMath;

/** Follows a given object using vision processing. */
public class FollowObject extends Command {

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;
    Object object;
    final double TURN_kP = 0.02;
    final double DIST_kP = 0.15;
    final double ANGLE_TOLERANCE = 3.0;
    final double DIST_TOLERANCE = 0.5;
    double firstDist, objectWidth;
    double distance = 1000; // super high to make sure command doesn't terminate
    boolean isFollowing = false;

    public enum Object {
        CARGO(), BAY_CLOSE, BAY_HIGH
    }

    public FollowObject(Object object) {
        requires(Robot.drivetrain);
        requires(Robot.limelight);
        this.object = object;
    }

    @Override
    protected void initialize() {

        if(object == Object.CARGO) {
            limelight.trackCargo();
            objectWidth = (13 / 12);
        } else if(object == Object.BAY_CLOSE) {
            limelight.trackRocketBayClose();
            objectWidth = (15 / 12);
        } else if(object == Object.BAY_HIGH) {
            limelight.trackRocketBayHigh();
            objectWidth = (15 / 12);
        }
        drivetrain.resetGyro();
        firstDist = limelight.getDistance(); // get initial dist away
        isFollowing = false;
    }

    @Override
    protected void execute() {

        // ensures that loop doesn't terminate if dist defaults to 0
        if(distance > 0) isFollowing = true; 

        drivetrain.resetGyro();

        double angleSetpoint = limelight.getHorizontalAngle();
        SmartDashboard.putNumber("Angle", angleSetpoint);
        distance = limelight.getDistance2(objectWidth) - 3.0; // stop 3 feet away
        SmartDashboard.putNumber("Dist", distance);

        // distance p loop
        double xSpeed = DIST_kP * distance;
        xSpeed = CSPMath.constrainKeepSign(xSpeed, 0.0, 1.0);
        System.out.println(xSpeed);

        // angle p loop
        // turn amount decreases as distance decreases
        double angleError = angleSetpoint - drivetrain.getGyroAngle();
        double turnOutput = TURN_kP * angleError;
        double zTurn = (Math.abs(angleError) > ANGLE_TOLERANCE) ? 
                CSPMath.constrainKeepSign(turnOutput, 0.25, 1.0) : 0;

        // command motor output
        drivetrain.arcade(xSpeed, zTurn, 1.0);
        
    }

    @Override
    protected boolean isFinished() {
      return (distance < DIST_TOLERANCE) && isFollowing;
    }

    @Override
    protected void end() {
        drivetrain.tank(0, 0, 0);
    }

    @Override
    protected void interrupted() {
    }
}
