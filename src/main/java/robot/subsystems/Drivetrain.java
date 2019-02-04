package robot.subsystems;

import robot.commands.drive.ManualDrive;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

public class Drivetrain extends Subsystem {

    // Device initialization
    private CANSparkMax leftMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax leftSlave1 = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax leftSlave2 = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightSlave1 = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightSlave2 = new CANSparkMax(0, MotorType.kBrushless);
    private CANEncoder leftEncoder = new CANEncoder(leftMotor);
    private CANEncoder rightEncoder = new CANEncoder(rightMotor);
    private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private DigitalInput lineSensorLeft = new DigitalInput(1); // yellow wire up
    private DigitalInput lineSensorMid = new DigitalInput(2);
    private DigitalInput lineSensorRight = new DigitalInput(3);
    private DoubleSolenoid gearShift = new DoubleSolenoid(0, 1);

    // Drive constants
    public final double MAX_VELOCITY = 9; // ft/s
    public final double MAX_ACCELERATION = 5; // ft/s^2
    public final double MAX_JERK = 190; // ft/s^3
    public final double WHEELBASE_WIDTH = 2; // ft
    public final double WHEEL_DIAMETER = (6.0 / 12.0); // ft
    public final double TICKS_PER_REV = 1.0; // neo
    public final double RAMP_RATE = 0.5; // seconds
    public final double ENCODER_TO_FEET = (1 / TICKS_PER_REV) * WHEEL_DIAMETER * Math.PI; // ft
    public final double DELTA_T = 0.02; // seconds

    // State vars
    private double fieldPosX, fieldPosY = 0;
    private boolean leftInverted, rightInverted;

    /** Constructs new Drivetrain object and configures devices. */
    public Drivetrain() {

        // Slave control
        leftSlave1.follow(leftMotor);
        leftSlave2.follow(leftMotor);
        rightSlave1.follow(rightMotor);
        rightSlave2.follow(rightMotor);

        // Reset
        reset();
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
        enableRampRate();
        setBrake();
        leftInverted = true;
        rightInverted = false;
        setInverted(false);
    }

    /** Sets left motors to given percentage (-1.0 - 1.0). */
    public void setLeft(double percent) {
        leftMotor.set(percent);
    }

    /** Sets right motors to given percentage (-1.0 - 1.0). */
    public void setRight(double percent) {
        rightMotor.set(percent);
    }

    /** Inverts drivetrain. True inverts each side from the
     *  current state set in the Drivetrain constructor. */
    public void setInverted(boolean isInverted) {
        setLeftInverted(isInverted);
        setRightInverted(isInverted);
    }

    /** Inverts the left side of the drivetrain. True inverts it
     *  from the current state set in the Drivetrain constructor. */
    public void setLeftInverted(boolean isInverted) {
        if(leftInverted) isInverted = !isInverted;
        leftMotor.setInverted(isInverted);
        leftSlave1.setInverted(isInverted);
        leftSlave2.setInverted(isInverted);
    }

    /** Inverts the right side of the drivetrain. True inverts it
     *  from the current state set in the Drivetrain constructor. */
    public void setRightInverted(boolean isInverted) {
        if(rightInverted) isInverted = !isInverted;
        rightMotor.setInverted(isInverted);
        rightSlave1.setInverted(isInverted);
        rightSlave2.setInverted(isInverted);
    }

    /** Sets drivetrain to brake mode. */
    public void setBrake() {
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftSlave1.setIdleMode(IdleMode.kBrake);
        leftSlave2.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightSlave1.setIdleMode(IdleMode.kBrake);
        rightSlave2.setIdleMode(IdleMode.kBrake);
    }

    /** Sets drivetrain to coast mode. */
    public void setCoast() {
        leftMotor.setIdleMode(IdleMode.kCoast);
        leftSlave1.setIdleMode(IdleMode.kCoast);
        leftSlave2.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);
        rightSlave1.setIdleMode(IdleMode.kCoast);
        rightSlave2.setIdleMode(IdleMode.kCoast);
    }

    /** Resets encoder values to 0 for both sides of drivetrain. */
    public void resetEncoders() {
    }

    /** Returns left encoder position in feet. */
    public double getLeftPosition() {
        return leftEncoder.getPosition() * ENCODER_TO_FEET;
    }

    /** Returns right encoder position in feet. */
    public double getRightPosition() {
        return rightEncoder.getPosition() * ENCODER_TO_FEET;
    }

    /** Returns encoder position in feet as average of left and right encoders. */
    public double getPosition() {
        return (getLeftPosition() + getRightPosition()) / 2;
    }

    /** Returns left encoder position in native Spark units (revolutions) */
    public double getRawLeftPosition() {
        return leftEncoder.getPosition();
    }

    /** Returns left encoder position in native Spark units (revolutions) */
    public double getRawRightPosition() {
        return rightEncoder.getPosition();
    }

    /** Returns left encoder velocity in feet per second. */
    public double getLeftVelocity() {
        return leftEncoder.getVelocity() * ENCODER_TO_FEET / 60.0; // native is rpm
    }

    /** Returns right encoder velocity in feet per second. */
    public double getRightVelocity() {
        return rightEncoder.getVelocity() * ENCODER_TO_FEET / 60.0; // native is rpm
    }

    /** Returns average robot velocity in feet per second. */
    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2;
    }

    /** Returns the left motor output as a percentage. */
    public double getLeftOutput() {
        return leftMotor.get();
    }

        /** Returns the right motor output as a percentage. */
    public double getRightOutput() {
        return rightMotor.get();
    }

    /** Returns average motor output current. */
    public double getMotorCurrent() {
        return (leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent()) / 2;
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

    /** Calibrates the gyro to reduce drifting. Only call when robot is not moving. */
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
        leftMotor.setRampRate(RAMP_RATE);
        rightMotor.setRampRate(RAMP_RATE);
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
        setLeft(leftInput * throttle);
        setRight(rightInput * throttle);

    }

    /** Controls drivetrain with tank model, individually moving left and
     *  right sides. Output multiplied by throttle. */
    public void tank(double leftSpeed, double rightSpeed, double throttle) {
        setLeft(leftSpeed * throttle);
        setRight(rightSpeed * throttle);
    }

}
