package robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import robot.utils.CSPMath;

/** Follows a given object using vision processing. */
public class FollowObject extends Command {

    public enum Object {
        CARGO, BAY_CLOSE, BAY_HIGH
    }

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    Object object;
    double objectWidth;
    double distance, initialDist, angleSetpoint = 1000; // super high to make sure command doesn't terminate
    boolean isFollowing, leftSense, midSense, rightSense = false;

    final double TURN_kP = 0.01;
    final double DIST_kP = 0.15;
    final double ANGLE_TOLERANCE = 3.0;
    final double DIST_TOLERANCE = 0.5;

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

        // reset
        drivetrain.resetGyro();
        isFollowing = false;

    }

    @Override
    protected void execute() {

        // ensures that loop doesn't terminate if dist defaults to 0
        if(distance > 0 && !isFollowing) {
            isFollowing = true;
            initialDist = distance;
        }

        // see if any line followers have detected line
        leftSense = drivetrain.getLeftLineSensor();
        midSense = drivetrain.getMidLineSensor();
        rightSense = drivetrain.getRightLineSensor();

        // reset gyro every loop to keep cam angle relative to bot angle
        drivetrain.resetGyro();

        // get angle and distance
        angleSetpoint = limelight.getHorizontalAngle();
        distance = limelight.getDistance(objectWidth) - (15 / 12); // stop 15 in away

        // distance p loop
        double xSpeed = DIST_kP * distance;
        xSpeed = CSPMath.constrainKeepSign(xSpeed, 0.0, 1.0);

        // angle p loop, turns less as distance shrinks
        double angleError = angleSetpoint - drivetrain.getGyroAngle();
        double turnAmount = distance / initialDist;
        double turnOutput = TURN_kP * angleError * turnAmount;
        double zTurn = (Math.abs(angleError) > ANGLE_TOLERANCE) ?
                CSPMath.constrainKeepSign(turnOutput, 0.15, 1.0) : 0;

        // command motor output
        drivetrain.arcade(xSpeed, zTurn, 1.0);
        
    }

    @Override
    protected boolean isFinished() {
        // end if dist and angle are within tolerance and it was following
        // or if any line followers sens a line
        return ((Math.abs(angleSetpoint) < ANGLE_TOLERANCE)
                && (Math.abs(distance) < DIST_TOLERANCE) && isFollowing)
                || (leftSense || midSense || rightSense);
    }

    @Override
    protected void end() {
        drivetrain.tank(0, 0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
