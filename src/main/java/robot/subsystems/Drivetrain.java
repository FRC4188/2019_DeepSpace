package robot.subsystems;

import robot.commands.drive.ManualDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Drivetrain extends Subsystem {

    // Device initialization
    private TalonSRX left = new TalonSRX(6);
    private TalonSRX leftSlave1 = new TalonSRX(5);
    private TalonSRX leftSlave2 = new TalonSRX(0);
    private TalonSRX right = new TalonSRX(7);
    private TalonSRX rightSlave1 = new TalonSRX(8);
    private TalonSRX rightSlave2 = new TalonSRX(0);
    private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private DigitalInput lineSensorLeft = new DigitalInput(1);
    private DigitalInput lineSensorMid = new DigitalInput(2);
    private DigitalInput lineSensorRight = new DigitalInput(3);

    // Drive constants
    private final double MAX_VELOCITY = 0; // ft/s
    private final double MAX_ACCELERATION = 0; // ft/s^2
    private final double MAX_JERK = 0; // ft/s^3
    private final double WHEELBASE_WIDTH = 0; // ft
    private final double WHEEL_DIAMETER = (0.0 / 12.0); // ft
    private final double TICKS_PER_REV = 4096; // talon units
    private final double RAMP_RATE = 0.5; // seconds
    private final double ENCODER_TO_FEET = (1 / TICKS_PER_REV) * WHEEL_DIAMETER * Math.PI; // ft
    private final double DELTA_T = 0.02; // seconds
    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    private final double kV = 0;
    private final double kA = 0;

    // State fields
    private double lastDriveError, lastTurnError, driveIntegral, turnIntegral = 0;
    private boolean isDriveFinished, isTurnFinished, isPathFinished = true;
    private boolean isFollowingLine = false;

    /** Constructs new Drivetrain object and configures devices */
    public Drivetrain() {

        // Slave control
        leftSlave1.follow(left);
        leftSlave2.follow(left);
        rightSlave1.follow(right);
        rightSlave2.follow(right);

        // Encoders
        left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        resetEncoders();

        // Drive config
        enableRampRate();
        setBrake();

        // Gyro
        gyro.reset();

    }

    /** Defines default command that will run when object is created */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualDrive());
    }

    /** Inverts drivetrain. */
    public void setInverted(boolean isInverted) {
        left.setInverted(isInverted);
        leftSlave1.setInverted(isInverted);
        leftSlave2.setInverted(isInverted);
        right.setInverted(isInverted);
        rightSlave1.setInverted(isInverted);
        rightSlave2.setInverted(isInverted);
    }

    /** Inverts the left side of the drivetrain. */
    public void setLeftInverted(boolean isInverted) {
        left.setInverted(isInverted);
        leftSlave1.setInverted(isInverted);
        leftSlave2.setInverted(isInverted);
    }

    /** Inverts the right side of the drivetrain. */
    public void setRightInverted(boolean isInverted) {
        right.setInverted(isInverted);
        rightSlave1.setInverted(isInverted);
        rightSlave2.setInverted(isInverted);
    }

    /** Sets drive talons to brake mode. */
    public void setBrake() {
        left.setNeutralMode(NeutralMode.Brake);
        leftSlave1.setNeutralMode(NeutralMode.Brake);
        leftSlave2.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);
        rightSlave1.setNeutralMode(NeutralMode.Brake);
        rightSlave2.setNeutralMode(NeutralMode.Brake);
    }

    /** Sets drive talons to coast mode. */
    public void setCoast() {
        left.setNeutralMode(NeutralMode.Coast);
        leftSlave1.setNeutralMode(NeutralMode.Coast);
        leftSlave2.setNeutralMode(NeutralMode.Coast);
        right.setNeutralMode(NeutralMode.Coast);
        rightSlave1.setNeutralMode(NeutralMode.Coast);
        rightSlave2.setNeutralMode(NeutralMode.Coast);
    }

    /** Resets encoder values to 0 for both sides of drivetrain. */
    public void resetEncoders() {
        left.setSelectedSensorPosition(0, 0, 10);
        right.setSelectedSensorPosition(0, 0, 10);
    }

    /** Returns left encoder position in feet. */
    public double getLeftPosition() {
        return left.getSelectedSensorPosition() * ENCODER_TO_FEET;
    }

    /** Returns right encoder position in feet. */
    public double getRightPosition() {
        return right.getSelectedSensorPosition() * ENCODER_TO_FEET;
    }

    /** Returns left encoder position in native talon units */
    public double getRawLeftPosition() {
        return left.getSelectedSensorPosition();
    }

    /** Returns right encoder position in native talon units. */
    public double getRawRightPosition() {
        return right.getSelectedSensorPosition();
    }

    /** Returns left encoder velocity in feet per second. */
    public double getLeftVelocity() {
        return left.getSelectedSensorVelocity() * ENCODER_TO_FEET * 10; // native talon is per 100ms
    }

    /** Returns right encoder velocity in feet per second. */
    public double getRightVelocity() {
        return right.getSelectedSensorVelocity() * ENCODER_TO_FEET * 10; // native talon is per 100ms
    }

    /** Enables open and closed loop ramp rate */
    public void enableRampRate() {
        left.configClosedloopRamp(RAMP_RATE);
        left.configOpenloopRamp(RAMP_RATE);
        right.configOpenloopRamp(RAMP_RATE);
        right.configClosedloopRamp(RAMP_RATE);
    }

    /** Controls drivetrain with arcade model, with positive xSpeed going forward
     *  and positive zTurn turning right. Output multiplied by throttle. */
    public void arcade(double xSpeed, double zTurn, double throttle) {

        double MAX_INPUT = 1.0;
        double turnRatio;
        double leftInput = xSpeed - zTurn;
        double rightInput = xSpeed + zTurn;

        // ensure that input does not exceed 1.0
        // if it does, reduce greatest input to 1.0 and reduce other proportionately
        if(leftInput > MAX_INPUT || rightInput > MAX_INPUT) {
            if(rightInput > leftInput) {
                turnRatio = leftInput / rightInput;
                rightInput = MAX_INPUT;
                leftInput = MAX_INPUT * turnRatio;
            } else if (leftInput > rightInput) {
                turnRatio = rightInput / leftInput;
                leftInput = MAX_INPUT;
                rightInput = MAX_INPUT * turnRatio;
            }
        } else if(leftInput < -MAX_INPUT || rightInput < -MAX_INPUT) {
            if(rightInput < leftInput) {
                turnRatio = leftInput / rightInput;
                rightInput = -MAX_INPUT;
                leftInput = -MAX_INPUT * turnRatio;
            } else if (leftInput < rightInput) {
                turnRatio = rightInput / leftInput;
                leftInput = -MAX_INPUT;
                rightInput = -MAX_INPUT * turnRatio;
            }
        }

        // command motor output
        left.set(ControlMode.PercentOutput, leftInput * throttle);
        right.set(ControlMode.PercentOutput, rightInput * throttle);
            
    }

    /** Controls drivetrain with tank model, individually moving left and
     *  right sides. Output multiplied by throttle. */
    public void tank(double leftSpeed, double rightSpeed, double throttle) {
        left.set(ControlMode.PercentOutput, leftSpeed * throttle);
        right.set(ControlMode.PercentOutput, rightSpeed * throttle);
    }

    /** Drives to distance (in feet) using PID loop. */
    public void driveToDistance(double distance, double tolerance) {
        isDriveFinished = false;
        double input = (getLeftPosition() + getRightPosition()) / 2; //avg encoder distance
        double error = distance - input;
        driveIntegral += error * DELTA_T;
        double driveDerivative = (error - lastDriveError) / DELTA_T;
        double output = kP * error + kI * driveIntegral + kD * driveDerivative;
        lastDriveError = error;
        tank(output, output, 1.0);
        if(error < tolerance) {
            isDriveFinished = true;
        }
    }

    /** Returns true when error on currently running driveToDistance is less than tolerance */
    public boolean isDriveFinished() {
        return isDriveFinished;
    }

    /** Resets state fields associated with driveToDistance. Should call before
     *  and/or after running driveToDistance(). */
    public void resetDriveToDistance() {
        lastDriveError = 0;
        driveIntegral = 0;
    }

    /** Turns to angle (in degrees) using PID loop. */
    public void turnToAngle(double angle, double tolerance) {
        isTurnFinished = false;
        double input = gyro.getAngle();
        double error = angle - input;
        turnIntegral += error * DELTA_T;
        double turnDerivative = (error - lastTurnError) / DELTA_T;
        double output = kP * error + kI * turnIntegral * kD * turnDerivative;
        lastTurnError = error;
        tank(output, -output, 1.0);
        if(error < tolerance) {
            isTurnFinished = true;
        }
    }

    /** Returns true when error on currently running turnToAngle is less than tolerance. */
    public boolean isTurnFinished() {
        return isTurnFinished;
    }

    /** Resets state fields associated with turnToAngle. Should call before
     *  and/or after running turnToAngle(). */
    public void resetTurnToAngle() {
        lastTurnError = 0;
        turnIntegral = 0;
    }

    /** Uses photosensors to detect reflective tape on ground and drive along line. */
    public void followLine() {

        // line following constants
        final double SPEED = 0.2;
        final double TURN_MINOR = 0.1;
        final double TURN_MAJOR = 0.3;

        // get data from photo sensors, true = reflecting
        boolean leftSense = lineSensorLeft.get();
        boolean midSense = lineSensorMid.get();
        boolean rightSense = lineSensorRight.get();
        
        // drive forward until line is detected, then begin control loop
        if(!isFollowingLine) {
            System.out.println("Not tracking line, continuing straight.");
            arcade(SPEED, 0, 1.0);
            if(leftSense || rightSense || midSense) isFollowingLine = true;
        } else {
            if(leftSense && !midSense && !rightSense) {         // left only
                System.out.println("Only left sensing, major turn right.");
                arcade(SPEED, -TURN_MAJOR, 1.0);
            } else if(leftSense && midSense && !rightSense) {   // left and mid
                System.out.println("Left and mid sensing, minor turn right.");
                arcade(SPEED, -TURN_MINOR, 1.0);
            } else if(!leftSense && midSense && !rightSense) {  // mid only
                System.out.println("Only mid sensing, continuing straight.");
                arcade(SPEED, 0, 1.0);
            } else if(!leftSense && midSense && rightSense) {   // right and mid
                System.out.println("Right and mid sensing, minor turn left.");
                arcade(SPEED, TURN_MINOR, 1.0);
            } else if(!leftSense && !midSense && rightSense) {  // right only
                System.out.println("Only right sensing, major turn left.");
                arcade(SPEED, TURN_MAJOR, 1.0);
            } else if(leftSense && !midSense && rightSense) {   // left and right
                System.out.println("Left and right sensing, continuing straight.");
                arcade(SPEED, 0, 1.0);
            } else if(leftSense && midSense && rightSense) {    // all three
                System.out.println("All three sensing, continuing straight.");
                arcade(SPEED, 0, 1.0);
            } else {
                System.out.println("Lost track of line, driving backwards.");
                arcade(-SPEED, 0, 1.0);
            }
        }
        
    }

    /** Resets state fields associated with followLine. Should call before 
     *  and/or after running followLine(). */
    public void resetFollowLine() {
        isFollowingLine = false;
    }

    /** Generates path from waypoints and returns array of EncoderFollowers
     *  for left (0) and right (1) sides of the path. */
    public EncoderFollower[] getEncoderFollowers(Waypoint[] points) {

        // create trajectory config
        Trajectory.Config config = new Trajectory.Config(
                Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, DELTA_T,
                MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);

        // generate trajectory + create followers
        Trajectory trajectory = Pathfinder.generate(points, config);
        TankModifier modifier = new TankModifier(trajectory).modify(WHEELBASE_WIDTH);
        EncoderFollower leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
        EncoderFollower rightFollower = new EncoderFollower(modifier.getRightTrajectory());

        // follower config
        leftFollower.configureEncoder(left.getSelectedSensorPosition(0),
                (int) TICKS_PER_REV, WHEEL_DIAMETER);
        rightFollower.configureEncoder(right.getSelectedSensorPosition(0),
                (int) TICKS_PER_REV, WHEEL_DIAMETER);
        leftFollower.configurePIDVA(kP, kI, kD, kV, kA);
        rightFollower.configurePIDVA(kP, kI, kD, kV, kA);

        // return new followers
        return new EncoderFollower[] {
            leftFollower,   // 0
            rightFollower   // 1
        };
    }

    /** Follows path given array of EncoderFollowers (left at index 0, right at 1).
     *  If isReversed is true, the drivetrain will be inverted. */
    public void followPath(EncoderFollower[] followers, boolean isReversed) {

        // resets state field
        isPathFinished = false;

        // invert drivetrain if needed
        if(isReversed) {
            setInverted(true);
        }

        // get followers from array
        EncoderFollower leftFollower = followers[0];
        EncoderFollower rightFollower = followers[1];

        // get motor setpoints
        double l = leftFollower.calculate(left.getSelectedSensorPosition(0));
        double r = rightFollower.calculate(right.getSelectedSensorPosition(0));

        // turn control loop (kP from 254 presentation)
        double gyroHeading = gyro.getAngle();
        double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
        double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
        double turn = 0.8 * (-1.0/80.0) * angleDifference;

        // use output
        tank(l + turn, r - turn, 1.0);

        // is it finished
        if(leftFollower.isFinished() || rightFollower.isFinished()) {
            isPathFinished = true;
            setInverted(false);
        }

    }

    /** Returns true when currently executing path is finished. */
    public boolean isPathFinished() {
        return isPathFinished;
    }

}
