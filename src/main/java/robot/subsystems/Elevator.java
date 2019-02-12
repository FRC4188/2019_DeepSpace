package robot.subsystems;

import robot.commands.elevator.ManualElevator;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends Subsystem {

    // Device initialization
    private CANSparkMax elevatorMotor = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax elevatorSlave = new CANSparkMax(12, MotorType.kBrushless);
    private CANEncoder elevatorEncoder = new CANEncoder(elevatorMotor);

    // Elevator constants
    private final double TICKS_PER_REV = 1; // neo
    private final double SPOOL_DIAMETER = 0; //feet
    private final double ENCODER_TO_FEET = (SPOOL_DIAMETER * Math.PI) / TICKS_PER_REV; // feet
    private final double RAMP_RATE = 0.2; // seconds
    public final double DELTA_T = 0.02; // seconds

    // State vars
    private boolean elevatorInverted;

    /** Constructs new Elevator object and configures devices */
    public Elevator() {

        // Slave control
        elevatorSlave.follow(elevatorMotor);

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
        elevatorMotor.set(percent);
    }

    /** Inverts the the elevator. */
    public void setInverted(boolean isInverted) {
        if(elevatorInverted) isInverted = !isInverted;
        elevatorMotor.setInverted(isInverted);
        elevatorSlave.setInverted(isInverted);
    }

    /** Sets sparks to brake mode - Only mode that should be used. */
    public void setBrake() {
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorSlave.setIdleMode(IdleMode.kBrake);
    }

    /** Resets encoder values to 0 for both shoulder and wrist. */
    public void resetEncoders() {
    }

    /** Returns elevator position in feet. */
    public double getPosition() {
        return elevatorEncoder.getPosition() * ENCODER_TO_FEET;
    }

    /** Returns elevator encoder position in native spark units (rotations). */
    public double getRawPosition() {
        return elevatorEncoder.getPosition();
    }

    /** Returns elevator encoder velocity in feet per second. */
    public double getVelocity() {
        return elevatorEncoder.getVelocity() * ENCODER_TO_FEET / 60; // native is rpm
    }

    /** Returns elevator motor output as a percentage. */
    public double getOutput() {
        return elevatorMotor.get();
    }

    /** Returns elevator motor output current. */
    public double getCurrent() {
        return elevatorMotor.getOutputCurrent();
    }

    /** Enables ramp rate. */
    public void enableRampRate() {
        elevatorMotor.setRampRate(RAMP_RATE);
    }

}
