package robot.subsystems;

import robot.commands.arm.ManualArm;
import robot.utils.CSPMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

public class Arm extends Subsystem {

    // Device initialization
    private CANSparkMax shoulderMotor = new CANSparkMax(21, MotorType.kBrushless);
    private CANSparkMax shoulderSlave = new CANSparkMax(22, MotorType.kBrushless);

    // Encoders
    private CANEncoder shoulderEncoder = new CANEncoder(shoulderMotor);

    // Manipulation constants
    private final double TICKS_PER_REV = 1.0; // neo
    private final double ENCODER_TO_DEGREES = 360 / TICKS_PER_REV; // degrees
    private final double RAMP_RATE = 0.5; // seconds
    private final double FEEDFORWARD = 0; // percent out
    public final double DELTA_T = 0.02; // seconds

    // State vars
    private boolean shoulderInverted;

    /** Constructs new Arm object and configures devices */
    public Arm() {

        // Slave control
        shoulderSlave.follow(shoulderMotor, true);

        // Reset
        reset();

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
        enableRampRate();
        setBrake();
        shoulderInverted = false;
        setInverted(false);
    }

    /** Sets shoulder motors to given percentage (-1.0, 1.0). */
    public void set(double percent) {
        // add dynamic feedforward to counteract gravity and linearize response
        double output = percent + FEEDFORWARD * Math.sin(Math.toRadians(getPosition()));
        shoulderMotor.set(output);
    }

    /** Inverts the the arm. */
    public void setInverted(boolean isInverted) {
        if(shoulderInverted) isInverted = !isInverted;
        shoulderMotor.setInverted(isInverted);
        shoulderSlave.setInverted(isInverted);
    }

    /** Sets shoulder to brake mode - Only mode that should be used. */
    public void setBrake() {
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        shoulderSlave.setIdleMode(IdleMode.kBrake);
    }

    /** Resets shoulder encoder value to 0. */
    public void resetEncoders() {
    }

    /** Returns left encoder position in degrees. */
    public double getPosition() {
        return shoulderEncoder.getPosition() * ENCODER_TO_DEGREES;
    }

    /** Returns shoulder encoder position in native Spark units (revolutions). */
    public double getRawPosition() {
        return shoulderEncoder.getPosition();
    }

    /** Returns shoulder encoder velocity in degrees per second. */
    public double getVelocity() {
        return shoulderEncoder.getVelocity() * ENCODER_TO_DEGREES / 60; // native is rpm
    }

    /** Returns shoulder motor output as a percentage. */
    public double getOutput() {
        return shoulderMotor.get();
    }

    /** Returns shoulder motor output current. */
    public double getCurrent() {
        return shoulderMotor.getOutputCurrent();
    }

    /** Enables open and closed loop ramp rate. */
    public void enableRampRate() {
        shoulderMotor.setOpenLoopRampRate(RAMP_RATE);
    }

}
