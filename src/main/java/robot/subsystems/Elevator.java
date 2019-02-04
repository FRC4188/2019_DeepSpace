package robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.commands.elevator.ManualElevator;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator extends Subsystem {

    // Device initialization
    private TalonSRX left = new TalonSRX(0);
    private TalonSRX right = new TalonSRX(0);

    // Elevator constants
    private final double TICKS_PER_REV = 4096; // talon units
    private final double ENCODER_TO_DEGREES = 360 / TICKS_PER_REV; // degrees
    private final double RAMP_RATE = 0.05; // seconds
    public final double DELTA_T = 0.02; // seconds

    // State vars
    private boolean elevatorInverted;

    /** Constructs new Elevator object and configures devices */
    public Elevator() {

        // Slave control
        left.follow(right);

        // Encoders
        right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        resetEncoders();

        // Arm config
        enableRampRate();
        setBrake();
        elevatorInverted = false;
        setInverted(false);

    }

    /** Defines default command that will run when object is created. */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualElevator());
    }

    /** Prints necessary info to the dashboard. */
    private void updateShufleboard() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
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
    public void set(double percent) {
        right.set(ControlMode.PercentOutput, percent);
    }

    /** Inverts the the elevator. */
    public void setInverted(boolean isInverted) {
        if(elevatorInverted) isInverted = !isInverted;
        right.setInverted(isInverted);
        left.setInverted(isInverted);
    }

    /** Sets arm talons brake mode - Only mode that should be used. */
    public void setBrake() {
        right.setNeutralMode(NeutralMode.Brake);
        left.setNeutralMode(NeutralMode.Brake);
    }

    /** Resets encoder values to 0 for both shoulder and wrist. */
    public void resetEncoders() {
        right.setSelectedSensorPosition(0, 0, 10);
    }

    /** Returns left encoder position in degrees. */
    public double getPosition() {
        return right.getSelectedSensorPosition() * ENCODER_TO_DEGREES;
    }

    /** Returns elevator encoder position in native talon units. */
    public double getRawPosition() {
        return right.getSelectedSensorPosition();
    }

    /** Returns elevator encoder velocity in degrees per second. */
    public double getVelocity() {
        return right.getSelectedSensorVelocity() * ENCODER_TO_DEGREES * 10; // native talon is per 100ms
    }

    /** Returns elevator motor output as a percentage. */
    public double getOutput() {
        return right.getMotorOutputPercent();
    }

    /** Returns elevator motor output current. */
    public double getCurrent() {
        return right.getOutputCurrent();
    }

    /** Enables open and closed loop ramp rate. */
    public void enableRampRate() {
        right.configClosedloopRamp(RAMP_RATE);
        right.configOpenloopRamp(RAMP_RATE);
    }

}