package robot.subsystems;

import robot.commands.intake.IntakeSpin;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem{
    // Device init
    private TalonSRX intakeSrx = new TalonSRX(0), intakeWrist = new TalonSRX(0);

    // Manipulation constants
    private final double TICKS_PER_REV = 4096; // talon units
    private final double ENCODER_TO_DEGREES = 360 / TICKS_PER_REV; // degrees
    private final double RAMP_RATE = 0.05; // seconds
    public final double DELTA_T = 0.02; // seconds

    // State vars
    private boolean intakeInverted, intakeWristInverted;

    /** Constructs intake objects and configures devices. */
    public Intake(){
        // Encoders
        intakeSrx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        resetEncoders();
        intakeWrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        resetEncoders();

        // Intake config
        enableRampRate();
        setBrake();
        intakeInverted = false;
        intakeWristInverted = false;
        setInverted(false);
    }

    @Override
    /** Default command, runs when object is created. */
    protected void initDefaultCommand() {
        setDefaultCommand(new IntakeSpin());
    }

    /** Prints info to dashboard. */
    private void updateShufleboard() {
        SmartDashboard.putNumber("Intake wrist pos", getIntakeWristPosition());
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

    /** Sets intake motors to given percentage (-1.0, 1.0) */
    public void setintakeSrx(double percent) {
        intakeSrx.set(ControlMode.PercentOutput, percent);
    }

    /** Sets intake wrist motors to given percentage (-1.0, 1.0) */
    public void setWrist(double percent) {
        intakeWrist.set(ControlMode.PercentOutput, percent);
    }

    /** Inverts intake. */
    public void setInverted(boolean isInverted) {
        setintakeInverted(isInverted);
        setIntakeWristInverted(isInverted);
    }

    /** Inverts the motors of the intake. */
    public void setintakeInverted(boolean isInverted) {
        if(intakeInverted) isInverted = !isInverted;
        intakeSrx.setInverted(isInverted);
    }

    /** Inverts the wrist of the intake. */
    public void setIntakeWristInverted(boolean isInverted) {
        if(intakeWristInverted) isInverted = !isInverted;
        intakeWrist.setInverted(isInverted);
    }

    /** Sets intake talons brake mode - Only mode that should be used. */
    public void setBrake() {
        intakeSrx.setNeutralMode(NeutralMode.Brake);
        intakeWrist.setNeutralMode(NeutralMode.Brake);
    }

    /** Resets encoder values to 0 for both intake and intake wrist. */
    public void resetEncoders() {
        intakeSrx.setSelectedSensorPosition(0, 0, 10);
        intakeWrist.setSelectedSensorPosition(0, 0, 10);
    }

    /** Returns left encoder position in degrees. */
    public double getIntakePosition() {
        return intakeSrx.getSelectedSensorPosition() * ENCODER_TO_DEGREES;
    }

    /** Returns intake wrist encoder position in degrees. */
    public double getIntakeWristPosition() {
        return intakeWrist.getSelectedSensorPosition() * ENCODER_TO_DEGREES;
    }

    /** Returns intake encoder position in native talon units. */
    public double getRawIntakePosition() {
        return intakeSrx.getSelectedSensorPosition();
    }

    /** Returns intake wrist encoder position in native talon units. */
    public double getRawIntakeWristPosition() {
        return intakeWrist.getSelectedSensorPosition();
    }

    /** Returns intake encoder velocity in degrees per second. */
    public double getIntakeVelocity() {
        return intakeSrx.getSelectedSensorVelocity() * ENCODER_TO_DEGREES * 10; // native talon is per 100ms
    }

    /** Returns intake wrist encoder velocity in degrees per second. */
    public double getIntakeWristVelocity() {
        return intakeWrist.getSelectedSensorVelocity() * ENCODER_TO_DEGREES * 10; // native talon is per 100ms
    }

    /** Returns intake motor output as a percentage. */
    public double getIntakeOutput() {
        return intakeSrx.getMotorOutputPercent();
    }

    /** Returns intake wrist motor output as a percentage. */
    public double getIntakeWristOutput() {
        return intakeWrist.getMotorOutputPercent();
    }

    /** Returns intake motor output current. */
    public double getIntakeCurrent() {
        return intakeSrx.getOutputCurrent();
    }

    /** Returns intake wrist motor output current. */
    public double getIntakeWristCurrent() {
        return intakeWrist.getOutputCurrent();
    }

    /** Enables open and closed loop ramp rate. */
    public void enableRampRate() {
        intakeSrx.configClosedloopRamp(RAMP_RATE);
        intakeSrx.configOpenloopRamp(RAMP_RATE);
        intakeWrist.configOpenloopRamp(RAMP_RATE);
        intakeWrist.configClosedloopRamp(RAMP_RATE);
    }

    /** Sets intakeSrx motors to intakeSrxSpeed and wrist motors to wristSpeed (-1.0 - 1.0).
     *  Output multiplied by throttle. */
    public void controlSpin(double intakeSpeed, double throttle) {
        intakeSrx.set(ControlMode.PercentOutput, intakeSpeed * throttle);
    }
    
    public void controlWrist(double intakeWristSpeed, double throttle) {
        intakeWrist.set(ControlMode.PercentOutput, intakeWristSpeed * throttle);
    }
}