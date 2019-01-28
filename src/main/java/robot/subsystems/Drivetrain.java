package robot.subsystems;

import robot.commands.drive.ManualDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

public class Drivetrain extends Subsystem {

    // Device initialization
    private WPI_TalonSRX left = new WPI_TalonSRX(6);
    private WPI_TalonSRX leftSlave1 = new WPI_TalonSRX(5);
    //private WPI_TalonSRX leftSlave2 = new WPI_TalonSRX(0);
    private WPI_TalonSRX right = new WPI_TalonSRX(7);
    private WPI_TalonSRX rightSlave1 = new WPI_TalonSRX(8);
    //private WPI_TalonSRX rightSlave2 = new WPI_TalonSRX(0);
    private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private DigitalInput lineSensorLeft = new DigitalInput(1); // yellow wire up
    private DigitalInput lineSensorMid = new DigitalInput(2);
    private DigitalInput lineSensorRight = new DigitalInput(3);
    private DoubleSolenoid gearShift = new DoubleSolenoid(0, 1);

    // Drive constants
    public final double MAX_VELOCITY = 0; // ft/s
    public final double MAX_ACCELERATION = 0; // ft/s^2
    public final double MAX_JERK = 0; // ft/s^3
    public final double WHEELBASE_WIDTH = 0; // ft
    public final double WHEEL_DIAMETER = (6.0 / 12.0); // ft
    public final double TICKS_PER_REV = 4096; // talon units
    public final double RAMP_RATE = 0.05; // seconds
    public final double ENCODER_TO_FEET = (1 / TICKS_PER_REV) * WHEEL_DIAMETER * Math.PI; // ft
    public final double DELTA_T = 0.02; // seconds

    // State vars
    private double fieldPosX, fieldPosY = 0;

    /** Constructs new Drivetrain object and configures devices. */
    public Drivetrain() {

        // Slave control
        leftSlave1.follow(left);
        //leftSlave2.follow(left);
        rightSlave1.follow(right);
        //rightSlave2.follow(right);

        // Encoders
        left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        resetEncoders();

        // Drive config
        enableRampRate();
        setBrake();
        setLeftInverted(true);

        // Gyro
        resetGyro();
        calibrateGyro();

    }

    /** Defines default command that will run when object is created. */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualDrive());
    }

    /** Prints necessary info to dashboard. */
    private void updateShuffleboard() {
        SmartDashboard.putNumber("L Pos", getLeftPosition());
        SmartDashboard.putNumber("R Pos", getRightPosition());
        SmartDashboard.putNumber("L Vel", getLeftVelocity());
        SmartDashboard.putNumber("R Vel", getRightVelocity());
        SmartDashboard.putNumber("Gyro", getGyroAngle());
        SmartDashboard.putNumber("Field X", getFieldPosX());
        SmartDashboard.putNumber("Field Y", getFieldPosY());
    }

    /** Runs every loop. */
    @Override
    public void periodic() {
        trackFieldPosition();
        updateShuffleboard();
    }

    /** Resets necessary drive devices. */
    public void reset() {
        resetEncoders();
        resetGyro();
        resetFieldPos();
    }

    /** Sets left motors to given percentage (-1.0 - 1.0). */
    public void setLeft(double percent) {
        left.set(ControlMode.PercentOutput, percent);
    }

    /** Sets right motors to given percentage (-1.0 - 1.0). */
    public void setRight(double percent) {
        right.set(ControlMode.PercentOutput, percent);
    }

    /** Inverts drivetrain. */
    public void setInverted(boolean isInverted) {
        left.setInverted(isInverted);
        leftSlave1.setInverted(isInverted);
        //leftSlave2.setInverted(isInverted);
        right.setInverted(isInverted);
        rightSlave1.setInverted(isInverted);
        //rightSlave2.setInverted(isInverted);
    }

    /** Inverts the left side of the drivetrain. */
    public void setLeftInverted(boolean isInverted) {
        left.setInverted(isInverted);
        leftSlave1.setInverted(isInverted);
        //leftSlave2.setInverted(isInverted);
    }

    /** Inverts the right side of the drivetrain. */
    public void setRightInverted(boolean isInverted) {
        right.setInverted(isInverted);
        rightSlave1.setInverted(isInverted);
        //rightSlave2.setInverted(isInverted);
    }

    /** Sets drive talons to brake mode. */
    public void setBrake() {
        left.setNeutralMode(NeutralMode.Brake);
        leftSlave1.setNeutralMode(NeutralMode.Brake);
        //leftSlave2.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);
        rightSlave1.setNeutralMode(NeutralMode.Brake);
        //rightSlave2.setNeutralMode(NeutralMode.Brake);
    }

    /** Sets drive talons to coast mode. */
    public void setCoast() {
        left.setNeutralMode(NeutralMode.Coast);
        leftSlave1.setNeutralMode(NeutralMode.Coast);
        //leftSlave2.setNeutralMode(NeutralMode.Coast);
        right.setNeutralMode(NeutralMode.Coast);
        rightSlave1.setNeutralMode(NeutralMode.Coast);
        //rightSlave2.setNeutralMode(NeutralMode.Coast);
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

    /** Returns encoder position in feet as average of left and right encoders. */
    public double getPosition() {
        return (getLeftPosition() + getRightPosition()) / 2;
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

    /** Returns average robot velocity in feet per second. */
    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2;
    }

    /** Returns the left motor output as a percentage. */
    public double getLeftOutput() {
        return left.getMotorOutputPercent();
    }

        /** Returns the right motor output as a percentage. */
    public double getRightOutput() {
        return right.getMotorOutputPercent();
    }

    /** Returns average motor output current. */
    public double getMotorCurrent() {
        return (left.getOutputCurrent() + right.getOutputCurrent()) / 2;
    }

    /** Returns gyro angle in degrees. */
    public double getGyroAngle() {
        return Pathfinder.boundHalfDegrees(gyro.getAngle());
    }

    /** Returns gyro rate in degrees per sec. */
    public double getGyroRate() {
        return gyro.getRate();
    }

    /** Resets gyro angle to 0. AVOID CALLING THIS. */
    public void resetGyro() {
        gyro.reset();
    }

    public void calibrateGyro() {
        gyro.calibrate();
    }

    /** Returns whether or not left photo sensor is reflecting. */
    public boolean getLeftLineSensor() {
        return lineSensorLeft.get();
    }

    /** Returns whether or not left photo sensor is reflecting. */
    public boolean getMidLineSensor() {
        return lineSensorMid.get();
    }

    /** Returns whether or not left photo sensor is reflecting. */
    public boolean getRightLineSensor() {
        return lineSensorRight.get();
    }

    /** Tracks robot position on field. To be called in robotPeriodic(). */
    public void trackFieldPosition() {
        fieldPosX += getVelocity() * Math.cos(Math.toRadians(getGyroAngle())) * DELTA_T;
        fieldPosY += getVelocity() * Math.sin(Math.toRadians(getGyroAngle())) * DELTA_T;
    }

    /** Returns estimate of robot's x (forward / reverse) location relative
     *  to starting point in feet. */
    public double getFieldPosX() {
        return fieldPosX;
    }

    /** Returns estimate of robot's y (left / right) location relative
     *  to starting point in feet. */
    public double getFieldPosY() {
        return fieldPosY;
    }

    /** Resets field position variables to 0. */
    public void resetFieldPos() {
        fieldPosX = 0;
        fieldPosY = 0;
    }

    /** Enables open and closed loop ramp rate */
    public void enableRampRate() {
        left.configClosedloopRamp(RAMP_RATE);
        left.configOpenloopRamp(RAMP_RATE);
        right.configOpenloopRamp(RAMP_RATE);
        right.configClosedloopRamp(RAMP_RATE);
    }

    /** Sets gear shift solenoid to given value. */
    public void shiftGear(Value value) {
        gearShift.set(value);
    }

    /** Controls drivetrain with arcade model, with positive xSpeed going forward
     *  and positive zTurn turning right. Output multiplied by throttle. */
    public void arcade(double xSpeed, double zTurn, double throttle) {

        double MAX_INPUT = 1.0;
        double turnRatio;
        double leftInput = xSpeed + zTurn;
        double rightInput = xSpeed - zTurn;

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

}
