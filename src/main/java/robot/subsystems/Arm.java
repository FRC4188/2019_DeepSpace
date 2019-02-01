package robot.subsystems;

import robot.commands.arm.ManualArm;
import robot.utils.CSPMath;
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

    // Manipulation constants
    private final double TICKS_PER_REV = 4096; // talon units
    private final double ENCODER_TO_DEGREES = 360 / TICKS_PER_REV; // degrees
    private final double RAMP_RATE = 0.05; // seconds
    private final double FEEDFORWARD = 0; // percent out
    public final double DELTA_T = 0.02; // seconds

    // State vars
    private boolean shoulderInverted;

    /** Constructs new Arm object and configures devices */
    public Arm() {

        // Slave control
        shoulderSlave.follow(shoulder);

        // Encoders
        shoulder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        resetEncoders();

        // Arm config
        enableRampRate();
        setBrake();
        shoulderInverted = false;
        setInverted(false);

    }

    /** Defines default command that will run when object is created. */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualArm());
    }

    /** Prints necessary info to the dashboard. */
    private void updateShufleboard() {
        SmartDashboard.putNumber("Shoulder pos", getPosition());
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

    /** Sets shoulder motors to given percentage (-1.0, 1.0). */
    public void set(double percent) {
        // add dynamic feedforward to counteract gravity and linearize response
        double output = percent + FEEDFORWARD * Math.cos(Math.toRadians(getPosition()));
        output = CSPMath.constrainKeepSign(output, 0, 1.0);
        shoulder.set(ControlMode.PercentOutput, output);
    }

    /** Inverts the the arm. */
    public void setInverted(boolean isInverted) {
        if(shoulderInverted) isInverted = !isInverted;
        shoulder.setInverted(isInverted);
        shoulderSlave.setInverted(isInverted);
    }

    /** Sets arm talons brake mode - Only mode that should be used. */
    public void setBrake() {
        shoulder.setNeutralMode(NeutralMode.Brake);
        shoulderSlave.setNeutralMode(NeutralMode.Brake);
    }

    /** Resets encoder values to 0 for both shoulder and wrist. */
    public void resetEncoders() {
        shoulder.setSelectedSensorPosition(0, 0, 10);
    }

    /** Returns left encoder position in degrees. */
    public double getPosition() {
        return shoulder.getSelectedSensorPosition() * ENCODER_TO_DEGREES;
    }

    /** Returns shoulder encoder position in native talon units. */
    public double getRawPosition() {
        return shoulder.getSelectedSensorPosition();
    }

    /** Returns shoulder encoder velocity in degrees per second. */
    public double getVelocity() {
        return shoulder.getSelectedSensorVelocity() * ENCODER_TO_DEGREES * 10; // native talon is per 100ms
    }

    /** Returns shoulder motor output as a percentage. */
    public double getOutput() {
        return shoulder.getMotorOutputPercent();
    }

    /** Returns shoulder motor output current. */
    public double getCurrent() {
        return shoulder.getOutputCurrent();
    }

    /** Enables open and closed loop ramp rate. */
    public void enableRampRate() {
        shoulder.configClosedloopRamp(RAMP_RATE);
        shoulder.configOpenloopRamp(RAMP_RATE);
    }

}
