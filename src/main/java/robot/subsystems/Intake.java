package robot.subsystems;

import robot.commands.intake.ManualWrist;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {

    // Device init
    private WPI_TalonSRX intake = new WPI_TalonSRX(0);
    private WPI_TalonSRX wrist = new WPI_TalonSRX(0);
    private DoubleSolenoid hatchSolenoid = new DoubleSolenoid(0, 1);

    // Manipulation constants
    private final double TICKS_PER_REV = 4096; // talon units
    private final double ENCODER_TO_DEGREES = 360 / TICKS_PER_REV; // degrees
    private final double RAMP_RATE = 0.05; // seconds
    public final double DELTA_T = 0.02; // seconds

    // State vars
    private boolean intakeInverted, wristInverted;

    /** Constructs intake object and configures devices. */
    public Intake(){

        // Encoders
        wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        // Reset
        reset();

    }

    @Override
    /** Default command, runs when object is created. */
    protected void initDefaultCommand() {
        setDefaultCommand(new ManualWrist());
    }

    /** Prints info to dashboard. */
    private void updateShufleboard() {
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
        enableRampRate();
        setBrake();
        intakeInverted = false;
        wristInverted = false;
        setInverted(false);
    }

    /** Sets intake motors to given percentage (-1.0, 1.0) */
    public void spinIntake(double percent) {
        intake.set(ControlMode.PercentOutput, percent);
    }

    /** Sets intake wrist motors to given percentage (-1.0, 1.0) */
    public void setWrist(double percent) {
        wrist.set(ControlMode.PercentOutput, percent);
    }

    /** Fires hatch solenoid in given direction. */
    public void fireHatchSolenoid(Value value) {
        hatchSolenoid.set(value);
    }

    /** Inverts intake. */
    public void setInverted(boolean isInverted) {
        setIntakeInverted(isInverted);
        setWristInverted(isInverted);
    }

    /** Inverts the motors of the intake. */
    public void setIntakeInverted(boolean isInverted) {
        if(intakeInverted) isInverted = !isInverted;
        intake.setInverted(isInverted);
    }

    /** Inverts the wrist of the intake. */
    public void setWristInverted(boolean isInverted) {
        if(wristInverted) isInverted = !isInverted;
        wrist.setInverted(isInverted);
    }

    /** Sets intake talons to brake mode. */
    public void setBrake() {
        intake.setNeutralMode(NeutralMode.Brake);
        wrist.setNeutralMode(NeutralMode.Brake);
    }

    /** Sets intake talons to coast mode. */
    public void setCoast() {
        intake.setNeutralMode(NeutralMode.Coast);
        wrist.setNeutralMode(NeutralMode.Coast);
    }

    /** Resets encoder values to 0 for both intake and intake wrist. */
    public void resetEncoders() {
        wrist.setSelectedSensorPosition(0, 0, 10);
    }

    /** Returns intake wrist encoder position in degrees. */
    public double getWristPosition() {
        return intake.getSelectedSensorPosition() * ENCODER_TO_DEGREES;
    }

    /** Returns intake wrist encoder position in native talon units. */
    public double getRawIntakeWristPosition() {
        return wrist.getSelectedSensorPosition();
    }

    /** Returns intake wrist encoder velocity in degrees per second. */
    public double getIntakeWristVelocity() {
        return wrist.getSelectedSensorVelocity() * ENCODER_TO_DEGREES * 10; // native talon is per 100ms
    }

    /** Returns intake motor output as a percentage. */
    public double getIntakeOutput() {
        return intake.getMotorOutputPercent();
    }

    /** Returns intake wrist motor output as a percentage. */
    public double getWristOutput() {
        return wrist.getMotorOutputPercent();
    }

    /** Returns intake motor output current. */
    public double getIntakeCurrent() {
        return intake.getOutputCurrent();
    }

    /** Returns intake wrist motor output current. */
    public double getWristCurrent() {
        return wrist.getOutputCurrent();
    }

    /** Enables open and closed loop ramp rate. */
    public void enableRampRate() {
        intake.configClosedloopRamp(RAMP_RATE);
        intake.configOpenloopRamp(RAMP_RATE);
        wrist.configOpenloopRamp(RAMP_RATE);
        wrist.configClosedloopRamp(RAMP_RATE);
    }

}
