package robot.subsystems;

import robot.commands.arm.ManipulateArm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Arm extends Subsystem {

    // Device initialization
    private TalonSRX shoulder = new TalonSRX(0);
    private TalonSRX shoulderSlave= new TalonSRX(0);
    private TalonSRX wrist = new TalonSRX(0);
    private TalonSRX wristSlave = new TalonSRX(0);

    // Manipulation constants
    private final double TICKS_PER_REV = 4096; // talon units
    private final double ENCODER_TO_DEGREES = 360 / TICKS_PER_REV; // degrees
    private final double RAMP_RATE = 0.05; // seconds
    public final double DELTA_T = 0.02; // seconds

    // State vars
    private boolean shoulderInverted, wristInverted;

    /** Constructs new Arm object and configures devices */
    public Arm() {

        // Slave control
        shoulderSlave.follow(shoulder);
        wristSlave.follow(wrist);

        // Encoders
        shoulder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        resetEncoders();

        // Arm config
        enableRampRate();
        setBrake();
        shoulderInverted = false;
        wristInverted = false;
        setInverted(false);

    }

    /** Defines default command that will run when object is created. */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new Manipulate());
    }

    /** Prints necessary info to the dashboard. */
    private void updateShufleboard() {
        SmartDashboard.putNumber("Shoulder pos", getShoulderPosition());
        SmartDashboard.putNumber("Wrist pos", getWristPosition());
    }

    /** Runs every loop. */
    @Override
    public void periodic() {
        updateShufleboard();
    }

    /** Resets necessary devices. */
    public void reset() {
        resetEncoders();
    }

    /** Sets shoulder motors to given percentage (-1.0, 1.0) */
    public void setShoulder(double percent) {
        shoulder.set(ControlMode.PercentOutput, percent);
    }

    /** Sets wrist motors to given percentage (-1.0, 1.0) */
    public void setWrist(double percent) {
        wrist.set(ControlMode.PercentOutput, percent);
    }

    /** Inverts arm. */
    public void setInverted(boolean isInverted) {
        setShoulderInverted(isInverted);
        setWristInverted(isInverted);
    }

    /** Inverts the shoulder of the arm. */
    public void setShoulderInverted(boolean isInverted) {
        if(shoulderInverted) isInverted = !isInverted;
        shoulder.setInverted(isInverted);
        shoulderSlave.setInverted(isInverted);
    }

    /** Inverts the wrist of the arm. */
    public void setWristInverted(boolean isInverted) {
        if(wristInverted) isInverted = !isInverted;
        wrist.setInverted(isInverted);
        wristSlave.setInverted(isInverted);
    }

    /** Sets arm talons brake mode - Only mode that should be used. */
    public void setBrake() {
        shoulder.setNeutralMode(NeutralMode.Brake);
        shoulderSlave.setNeutralMode(NeutralMode.Brake);
        wrist.setNeutralMode(NeutralMode.Brake);
        wristSlave.setNeutralMode(NeutralMode.Brake);
    }

    /** Resets encoder values to 0 for both shoulder and wrist. */
    public void resetEncoders() {
        shoulder.setSelectedSensorPosition(0, 0, 10);
        wrist.setSelectedSensorPosition(0, 0, 10);
    }

    /** Returns left encoder position in degrees. */
    public double getShoulderPosition() {
        return shoulder.getSelectedSensorPosition() * ENCODER_TO_DEGREES;
    }

    /** Returns wrist encoder position in degrees. */
    public double getWristPosition() {
        return wrist.getSelectedSensorPosition() * ENCODER_TO_DEGREES;
    }

    /** Returns shoulder encoder position in native talon units. */
    public double getRawShoulderPosition() {
        return shoulder.getSelectedSensorPosition();
    }

    /** Returns wrist encoder position in native talon units. */
    public double getRawWristPosition() {
        return wrist.getSelectedSensorPosition();
    }

    /** Returns shoulder encoder velocity in degrees per second. */
    public double getShoulderVelocity() {
        return shoulder.getSelectedSensorVelocity() * ENCODER_TO_DEGREES * 10; // native talon is per 100ms
    }

    /** Returns wrist encoder velocity in degrees per second. */
    public double getWristVelocity() {
        return wrist.getSelectedSensorVelocity() * ENCODER_TO_DEGREES * 10; // native talon is per 100ms
    }

    /** Returns shoulder motor output as a percentage. */
    public double getShoulderOutput() {
        return shoulder.getMotorOutputPercent();
    }

    /** Returns wrist motor output as a percentage. */
    public double getWristOutput() {
        return wrist.getMotorOutputPercent();
    }

    /** Returns shoulder motor output current. */
    public double getShoulderCurrent() {
        return shoulder.getOutputCurrent();
    }

    /** Returns wrist motor output current. */
    public double getWristCurrent() {
        return wrist.getOutputCurrent();
    }

    /** Enables open and closed loop ramp rate. */
    public void enableRampRate() {
        shoulder.configClosedloopRamp(RAMP_RATE);
        shoulder.configOpenloopRamp(RAMP_RATE);
        wrist.configOpenloopRamp(RAMP_RATE);
        wrist.configClosedloopRamp(RAMP_RATE);
    }

    /** Sets shoulder motors to shoulderSpeed and wrist motors to wristSpeed (-1.0 - 1.0).
     *  Output multiplied by throttle. */
    public void control(double shoulderSpeed, double wristSpeed, double throttle) {
        shoulder.set(ControlMode.PercentOutput, shoulderSpeed * throttle);
        wrist.set(ControlMode.PercentOutput, wristSpeed * throttle);
    }

}
