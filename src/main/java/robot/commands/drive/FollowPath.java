package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

/** Follows given path of waypoints using Pathfinder library.
 *  If isReversed == true, the path will run backwards. */
public class FollowPath extends Command {

    public enum Path {
        TO_PERPENDICULAR,
        L_TO_L_FAR_ROCKET,
        L_TO_L_FAR_SHIP,
        M_TO_L_FAR_ROCKET,
        M_TO_L_FAR_SHIP,
        M_TO_R_FAR_ROCKET,
        M_TO_R_FAR_SHIP,
        R_TO_R_FAR_ROCKET,
        R_TO_R_FAR_SHIP
    }

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    private Notifier notif = new Notifier(() -> follow());
    private DistanceFollower leftFollower, rightFollower;
    private boolean isReversed, fromFile, isFinished;
    private double initialLeftDist, initialRightDist;
    private Waypoint[] points;
    private Path path;

    private final double kP = drivetrain.kP;
    private final double kI = drivetrain.kI;
    private final double kD = drivetrain.kD;
    private final double kV = drivetrain.kV;
    private final double kA = drivetrain.kA;
    private final double DELTA_T = 0.02; // seconds

  /** Follows path from given waypoints. isReversed causes the path
     * to be followed in reverse. */
    public FollowPath(Waypoint[] points, boolean isReversed) {
        requires(drivetrain);
        this.points = points;
        this.isReversed = isReversed;
        this.fromFile = false;
    }

    public FollowPath(Path path, boolean isReversed) {
        requires(drivetrain);
        this.isReversed = isReversed;
        this.path = path;
        if(path == Path.TO_PERPENDICULAR) this.fromFile = false;
        else this.fromFile = true;
    }

    @Override
    protected void initialize() {

        // initial vals
        initialLeftDist = drivetrain.getLeftPosition();
        initialRightDist = drivetrain.getRightPosition();
        isFinished = false;

        // create waypoints if going to perpendicular
        if(path == Path.TO_PERPENDICULAR) {

            /*
            // get necessary info from camera
            double currentAngle = Math.toRadians(drivetrain.getGyroAngle());
            double turnAngle = Math.toRadians(limelight.solvePerpendicular()[0]);
            double driveDist = limelight.solvePerpendicular()[1];
            double x = driveDist * Math.cos(turnAngle);
            double y = driveDist * Math.sin(turnAngle);
            double targetAngle = Math.toRadians(limelight.solvePerpendicular()[2]);

            SmartDashboard.putNumber("x drive", x);
            SmartDashboard.putNumber("y drive", y);

            // create points
            points = new Waypoint[] {
                new Waypoint(0, 0, currentAngle),
                new Waypoint(x, y, targetAngle)
            };*/
            final double PERP_LENGTH = 4.0;
            double[] distances = limelight.getDistance3d();
            double robotAngle = limelight.getRobotAngle();
            SmartDashboard.putNumber("robot angle", robotAngle);
            double X = distances[0];
            double Y = distances[1];
            double diag = Math.sqrt(X * X + Y * Y);
            double relAngleRad = Math.toRadians(limelight.getHorizontalAngle());
            double horizontalDistance = (diag - PERP_LENGTH * (X / Math.tan(relAngleRad) + Y) / diag)
                    * Math.sin(relAngleRad);
            double forwardDistance = -horizontalDistance
                    * Math.tan(Math.PI / 2 - Math.atan2(Y, X) + Math.atan2(Y - PERP_LENGTH, X) + relAngleRad);
            SmartDashboard.putNumber("global forward distance", Y);
            SmartDashboard.putNumber("global side distance", X);
            SmartDashboard.putNumber("forward distance", forwardDistance);
            SmartDashboard.putNumber("side distance", horizontalDistance);

            // create points
            points = new Waypoint[] {
                    new Waypoint(0, 0, 0),
                    new Waypoint(forwardDistance, horizontalDistance, Math.toRadians(robotAngle)) };

        }

        // generate points if not from file
        if(!fromFile) {

            // generate trajectory
            Trajectory.Config config = new Trajectory.Config(
                    Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, DELTA_T,
                    drivetrain.MAX_VELOCITY, drivetrain.MAX_ACCELERATION, drivetrain.MAX_JERK);
            Trajectory trajectory = Pathfinder.generate(points, config);

            // create followers
            TankModifier modifier = new TankModifier(trajectory).modify(drivetrain.WHEELBASE_WIDTH);
            leftFollower = new DistanceFollower(modifier.getLeftTrajectory());
            rightFollower = new DistanceFollower(modifier.getRightTrajectory());

        } else {

            // get left and right trajectories (swapped due to pathweaver bug)
            Trajectory leftTrajectory = PathfinderFRC.getTrajectory(path.toString() + ".right");
            Trajectory rightTrajectory = PathfinderFRC.getTrajectory(path.toString() + ".left");

            // create followers
            leftFollower = new DistanceFollower(leftTrajectory);
            rightFollower = new DistanceFollower(rightTrajectory);

        }

        // follower config
        leftFollower.configurePIDVA(kP, kI, kD, kV, kA);
        rightFollower.configurePIDVA(kP, kI, kD, kV, kA);

        // start
        notif.startPeriodic(DELTA_T);

    }

    protected void follow() {

        System.out.println("******FOLLOWING******");

        double leftPos = drivetrain.getLeftPosition() - initialLeftDist;
        double rightPos = drivetrain.getRightPosition() - initialRightDist;

        // invert drivetrain if needed
        if(isReversed) {
            drivetrain.setInverted(true);
            leftPos *= -1;
            rightPos *= -1;
        }

        // get motor setpoints
        double l = leftFollower.calculate(leftPos);
        double r = rightFollower.calculate(rightPos);

        // turn control loop (kP from 254 presentation)
        double gyroHeading = drivetrain.getGyroAngle();
        double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
        double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
        double turn = 0.8 * (1.0/80.0) * angleDifference;

        // use output
        drivetrain.tank(l + turn, r - turn);

        // determine if finished
        if(leftFollower.isFinished() && rightFollower.isFinished() && Math.abs(angleDifference) < 3.0) {
            isFinished = true;
        }

    }

    @Override
    protected boolean isFinished() {
        return isFinished;
    }

    @Override
    protected void end() {
        drivetrain.tank(0, 0);
        drivetrain.setInverted(false);
        notif.stop();
    }

    @Override
    protected void interrupted() {
        end();
    }

}
