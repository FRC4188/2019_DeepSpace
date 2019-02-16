package robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import robot.utils.CSPMath;

/** Follows a given object using vision processing. */
public class FollowObject extends Command {

    public enum Object {
        CARGO, BAY, BAY_3D
    }

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    // super high to make sure command doesn't terminate
    double distance, initialDist, distErr, angleSetpoint, angleErr = 1000;
    boolean isFollowing, leftSense, midSense, rightSense = false;
    Object object;

    final double TURN_kP = 0.01;
    final double DIST_kP = 0.15;
    final double ANGLE_TOLERANCE = 3.0;
    final double DIST_TOLERANCE = 0.5;

    double perpLength;
    boolean closerThanTarget = false;

    public FollowObject(Object object, double perpLength) {
        requires(Robot.drivetrain);
        requires(Robot.limelight);
        this.object = object;
        this.perpLength = perpLength;
    }

    @Override
    protected void initialize() {

        if(object == Object.CARGO) {
            limelight.trackCargo();
        } else if(object == Object.BAY) {
            limelight.trackBay();
        } else if(object == Object.BAY_3D){
            limelight.trackBay3D();
        }

        // reset
        isFollowing = false;
        closerThanTarget = perpLength > limelight.getDistance(limelight.getPipeline().getHeight());
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

        // get angle and distance
        angleSetpoint = limelight.getHorizontalAngle() + drivetrain.getGyroAngle();
        distance = limelight.getDistance(limelight.getPipeline().getHeight());
        distErr = distance - perpLength; // stop distanceToTarget away
        if(distErr < 0) distErr = 0;

        // distance p loop
        double xSpeed = DIST_kP * distErr;
        xSpeed = CSPMath.constrainKeepSign(xSpeed, 0.0, 1.0);

        // angle p loop, turns less as distance shrinks
        angleErr = angleSetpoint - drivetrain.getGyroAngle();
        //double distReducer = distance / initialDist;
        double turnOutput = TURN_kP * angleErr;
        double zTurn = (Math.abs(angleErr) > ANGLE_TOLERANCE) ?
                CSPMath.constrainKeepSign(turnOutput, 0.15, 1.0) : 0;

        // command motor output
        drivetrain.arcade(xSpeed, zTurn, 1.0);

        // debugging
        if(!(Math.abs(angleErr) < ANGLE_TOLERANCE)) System.out.println("turning" + angleErr);
        if(!(Math.abs(distErr) < DIST_TOLERANCE)) System.out.println("driving " + distErr);

    }

    @Override
    protected boolean isFinished() {
        // end if dist and angle are within tolerance and it was following
        // or if any line followers sense a line
        // or if the robot is already close to the target
        return ((Math.abs(angleErr) < ANGLE_TOLERANCE)
                && (Math.abs(distErr) < DIST_TOLERANCE) && isFollowing)
                || (leftSense || midSense || rightSense) || closerThanTarget;
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
