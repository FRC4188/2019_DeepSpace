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
    private TalonSRX elevatorMotor = new TalonSRX(0);
    private TalonSRX elevatorSlave = new TalonSRX(0);

    // Elevator constants
    private final double TICKS_PER_REV = 4096; // talon units
    private final double ENCODER_TO_DEGREES = 360 / TICKS_PER_REV; // degrees
    private final double RAMP_RATE = 0.2; // seconds
    public final double DELTA_T = 0.02; // seconds

    // State vars
    private boolean elevatorInverted;

    /** Constructs new Elevator object and configures devices */
    public Elevator() {

        // Slave control
        elevatorSlave.follow(elevatorMotor);

        // Encoders
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        // Reset
        reset();

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
        enableRampRate();
        setBrake();
        elevatorInverted = false;
        setInverted(false);
    }

    /** Sets shoulder motors to given percentage (-1.0, 1.0) */
    public void set(double percent) {
        elevatorMotor.set(ControlMode.PercentOutput, percent);
    }

    /** Inverts the the elevator. */
    public void setInverted(boolean isInverted) {
        if(elevatorInverted) isInverted = !isInverted;
        elevatorMotor.setInverted(isInverted);
        elevatorSlave.setInverted(isInverted);
    }

    /** Sets arm talons brake mode - Only mode that should be used. */
    public void setBrake() {
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        elevatorSlave.setNeutralMode(NeutralMode.Brake);
    }

    /** Resets encoder values to 0 for both shoulder and wrist. */
    public void resetEncoders() {
        elevatorMotor.setSelectedSensorPosition(0, 0, 10);
    }

    /** Returns left encoder position in degrees. */
    public double getPosition() {
        return elevatorMotor.getSelectedSensorPosition() * ENCODER_TO_DEGREES;
    }

    /** Returns elevator encoder position in native talon units. */
    public double getRawPosition() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    /** Returns elevator encoder velocity in degrees per second. */
    public double getVelocity() {
        return elevatorMotor.getSelectedSensorVelocity() * ENCODER_TO_DEGREES * 10; // native talon is per 100ms
    }

    /** Returns elevator motor output as a percentage. */
    public double getOutput() {
        return elevatorMotor.getMotorOutputPercent();
    }

    /** Returns elevator motor output current. */
    public double getCurrent() {
        return elevatorMotor.getOutputCurrent();
    }

    /** Enables open and closed loop ramp rate. */
    public void enableRampRate() {
        elevatorMotor.configClosedloopRamp(RAMP_RATE);
        elevatorMotor.configOpenloopRamp(RAMP_RATE);
    }

}
