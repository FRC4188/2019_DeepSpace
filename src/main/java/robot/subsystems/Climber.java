package robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber extends Subsystem {

    // Device initialization
    private WPI_TalonSRX climberFront = new WPI_TalonSRX(41);
    private WPI_TalonSRX climberFrontSlave = new WPI_TalonSRX(42);
    private WPI_TalonSRX climberRear = new WPI_TalonSRX(43);
    private WPI_TalonSRX climberRearSlave = new WPI_TalonSRX(44);

    // Constants
    private final double RAMP_RATE = 0.2; // seconds

    // State variables
    private boolean frontInverted, rearInverted;

    /** Constructs new Climber object and configures devices */
    public Climber() {

        // Slave control
        climberFrontSlave.follow(climberFront);
        climberRearSlave.follow(climberRear);

        // Reset
        reset();

    }

    /** Defines default command that will run when object is created. */
    @Override
    public void initDefaultCommand() {
    }

    /** Prints necessary info to the dashboard. */
    private void updateShufleboard() {
    }

    /** Runs every loop. */
    @Override
    public void periodic() {
        updateShufleboard();
    }

    /** Resets necessary devices. */
    public void reset() {
        enableRampRate();
        setBrake();
        frontInverted = false;
        rearInverted = false;
        setInverted(false);
    }

    /** Sets front climber motors to given percentage (-1.0, 1.0) */
    public void setFront(double percent) {
        climberFront.set(percent);
    }

    /** Sets rear climber motors to given percentage (-1.0, 1.0) */
    public void setRear(double percent) {
        climberRear.set(percent);
    }

    /** Inverts the the climber. */
    public void setInverted(boolean isInverted) {
        setFrontInverted(isInverted);
        setRearInverted(isInverted);
    }

    /** Inverts front climber talons. */
    public void setFrontInverted(boolean isInverted) {
        if(frontInverted) isInverted = !isInverted;
        climberFront.setInverted(isInverted);
        climberFrontSlave.setInverted(isInverted);
    }

    /** Inverts rear climber talons. */
    public void setRearInverted(boolean isInverted) {
        if(rearInverted) isInverted = !isInverted;
        climberRear.setInverted(isInverted);
        climberRearSlave.setInverted(isInverted);
    }

    /** Sets Talons to brake mode - Only mode that should be used. */
    public void setBrake() {
        climberFront.setNeutralMode(NeutralMode.Brake);
        climberFrontSlave.setNeutralMode(NeutralMode.Brake);
        climberRear.setNeutralMode(NeutralMode.Brake);
        climberRearSlave.setNeutralMode(NeutralMode.Brake);
    }

    /** Returns front climber motor output as a percentage. */
    public double getFrontOutput() {
        return climberFront.get();
    }

    /** Returns rear climber motor output as a percentage. */
    public double getRearOutput() {
        return climberRear.get();
    }

    /** Returns front climber motor output current. */
    public double getFrontCurrent() {
        return climberFront.getOutputCurrent();
    }

    /** Returns rear climber motor output current. */
    public double getRearCurrent() {
        return climberRear.getOutputCurrent();
    }

    /** Enables ramp rate. */
    public void enableRampRate() {
        climberFront.configOpenloopRamp(RAMP_RATE);
        climberFrontSlave.configOpenloopRamp(RAMP_RATE);
        climberRear.configOpenloopRamp(RAMP_RATE);
        climberRearSlave.configOpenloopRamp(RAMP_RATE);
        climberFront.configClosedloopRamp(RAMP_RATE);
        climberFrontSlave.configClosedloopRamp(RAMP_RATE);
        climberRear.configClosedloopRamp(RAMP_RATE);
        climberRearSlave.configClosedloopRamp(RAMP_RATE);
    }

}
