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
import jaci.pathfinder.followers.EncoderFollower;
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

    EncoderFollower leftFollower, rightFollower;
    boolean isReversed, fromFile, isFinished;
    double initialAngle;
    Notifier notif;
    Waypoint[] points;
    Path path;

    final double kP = 0.00005;
    final double kI = 0;
    final double kD = 0;
    final double kV = 1.0 / drivetrain.MAX_VELOCITY;
    final double kA = 0;
    final double DELTA_T = 0.02; // seconds

    /** Follows path from given waypoints. isReversed causes the path
     *  to be followed in reverse. */
    public FollowPath(Waypoint[] points, boolean isReversed) {
        requires(drivetrain);
        this.points = points;
        this.isReversed = isReversed;
        this.fromFile = false;
    }

    /** Follows given path. isReversed causes the path to be followed
     *  in reverse. */
    public FollowPath(Path path, boolean isReversed) {
        requires(drivetrain);
        setName("FollowPath: " + path.toString());
        this.isReversed = isReversed;
        this.path = path;
        if(path == Path.TO_PERPENDICULAR) this.fromFile = false;
        else this.fromFile = true;
    }

    @Override
    protected void initialize() {

        // save initial gyro angle to make all angles relative to first
        initialAngle = drivetrain.getGyroAngle();

        // create waypoints if going to perpendicular
        if(path == Path.TO_PERPENDICULAR) {

            // get necessary info from camera
            double initialAngleInRad = Math.toRadians(initialAngle);
            double turnAngle = Math.toRadians(limelight.solvePerpendicular()[0]);
            double driveDist = limelight.solvePerpendicular()[1] * 0.9;
            double x = driveDist * Math.cos(turnAngle);
            double y = driveDist * Math.sin(turnAngle);
            double targetAngle = Math.toRadians(limelight.solvePerpendicular()[2]);

            SmartDashboard.putNumber("Path X", x);
            SmartDashboard.putNumber("Path Y", y);

            // create points
            points = new Waypoint[] {
                new Waypoint(0, 0, initialAngleInRad),
                new Waypoint(x, y, targetAngle)
            };

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
            leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
            rightFollower = new EncoderFollower(modifier.getRightTrajectory());

        } else {

            // get left and right trajectories (swapped due to pathweaver bug)
            Trajectory leftTrajectory = PathfinderFRC.getTrajectory(path.toString() + ".right");
            Trajectory rightTrajectory = PathfinderFRC.getTrajectory(path.toString() + ".left");

            // create followers
            leftFollower = new EncoderFollower(leftTrajectory);
            rightFollower = new EncoderFollower(rightTrajectory);

        }

        // follower config
        leftFollower.configureEncoder((int) drivetrain.getRawLeftPosition(),
                (int) drivetrain.TICKS_PER_REV, drivetrain.WHEEL_DIAMETER);
        rightFollower.configureEncoder((int) drivetrain.getRawRightPosition(),
                (int) drivetrain.TICKS_PER_REV, drivetrain.WHEEL_DIAMETER);
        leftFollower.configurePIDVA(kP, kI, kD, kV, kA);
        rightFollower.configurePIDVA(kP, kI, kD, kV, kA);

        // start notifier
        notif = new Notifier(() -> follow());
        notif.startPeriodic(DELTA_T);

    }

    protected void follow() {

        // invert drivetrain if needed
        if(isReversed) drivetrain.setInverted(true);

        // get motor setpoints
        double l = leftFollower.calculate((int) drivetrain.getRawLeftPosition());
        double r = rightFollower.calculate((int) drivetrain.getRawRightPosition());

        // turn control loop (kP from 254 presentation)
        double gyroHeading = drivetrain.getGyroAngle() - initialAngle;
        double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
        double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
        double turn = 0.4 * (1.0/80.0) * angleDifference;
        System.out.println("angleDiff: " + angleDifference + " turn val: " + turn);

        // use output
        drivetrain.tank(l + turn, r - turn, 1.0);

        // determine if finished
        isFinished = leftFollower.isFinished() && rightFollower.isFinished();
        if(isFinished) notif.stop();

    }

    @Override
    protected boolean isFinished() {
        return leftFollower.isFinished() && rightFollower.isFinished();
    }

    @Override
    protected void end() {
        drivetrain.tank(0, 0, 0);
        drivetrain.setInverted(false);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
