package robot.subsystems;

import robot.commands.drive.ManualDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Drivetrain extends Subsystem {

    // Device initialization
    private TalonSRX left = new TalonSRX(6);
    private TalonSRX leftSlave1 = new TalonSRX(5);
    //private TalonSRX leftSlave2 = new TalonSRX(0);
    private TalonSRX right = new TalonSRX(7);
    private TalonSRX rightSlave1 = new TalonSRX(8);
    //private TalonSRX rightSlave2 = new TalonSRX(0);
    private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private DigitalInput lineSensorLeft = new DigitalInput(1);
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

    /** Constructs new Drivetrain object and configures devices */
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

    /** Defines default command that will run when object is created */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualDrive());
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
        return gyro.getAngle();
    }

    /** Returns gyro rate in degrees per sec. */
    public double getGyroRate() {
        return gyro.getRate();
    }

    /** Resets gyro angle to 0. */
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
