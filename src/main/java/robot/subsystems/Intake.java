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
    private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(0);
    private WPI_TalonSRX wristMotor = new WPI_TalonSRX(0);
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
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

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
        intakeMotor.set(ControlMode.PercentOutput, percent);
    }

    /** Sets intake wrist motors to given percentage (-1.0, 1.0) */
    public void setWrist(double percent) {
        wristMotor.set(ControlMode.PercentOutput, percent);
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
        intakeMotor.setInverted(isInverted);
    }

    /** Inverts the wrist of the intake. */
    public void setWristInverted(boolean isInverted) {
        if(wristInverted) isInverted = !isInverted;
        wristMotor.setInverted(isInverted);
    }

    /** Sets intake talons to brake mode. */
    public void setBrake() {
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setNeutralMode(NeutralMode.Brake);
    }

    /** Sets intake talons to coast mode. */
    public void setCoast() {
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        wristMotor.setNeutralMode(NeutralMode.Coast);
    }

    /** Resets encoder values to 0 for both intake and intake wrist. */
    public void resetEncoders() {
        wristMotor.setSelectedSensorPosition(0, 0, 10);
    }

    /** Returns intake wrist encoder position in degrees. */
    public double getWristPosition() {
        return intakeMotor.getSelectedSensorPosition() * ENCODER_TO_DEGREES;
    }

    /** Returns intake wrist encoder position in native talon units. */
    public double getRawIntakeWristPosition() {
        return wristMotor.getSelectedSensorPosition();
    }

    /** Returns intake wrist encoder velocity in degrees per second. */
    public double getIntakeWristVelocity() {
        return wristMotor.getSelectedSensorVelocity() * ENCODER_TO_DEGREES * 10; // native talon is per 100ms
    }

    /** Returns intake motor output as a percentage. */
    public double getIntakeOutput() {
        return intakeMotor.getMotorOutputPercent();
    }

    /** Returns intake wrist motor output as a percentage. */
    public double getWristOutput() {
        return wristMotor.getMotorOutputPercent();
    }

    /** Returns intake motor output current. */
    public double getIntakeCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    /** Returns intake wrist motor output current. */
    public double getWristCurrent() {
        return wristMotor.getOutputCurrent();
    }

    /** Enables open and closed loop ramp rate. */
    public void enableRampRate() {
        intakeMotor.configClosedloopRamp(RAMP_RATE);
        intakeMotor.configOpenloopRamp(RAMP_RATE);
        wristMotor.configOpenloopRamp(RAMP_RATE);
        wristMotor.configClosedloopRamp(RAMP_RATE);
    }

}
