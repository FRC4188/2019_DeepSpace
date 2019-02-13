package robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber extends Subsystem {

    // Device initialization
    private WPI_TalonSRX climberLeft = new WPI_TalonSRX(41);
    private WPI_TalonSRX climberRight = new WPI_TalonSRX(42);
    
    //private WPI_TalonSRX leftSlave = new WPI_TalonSRX(13);
    //private WPI_TalonSRX rightSlave = new WPI_TalonSRX(14);

    // Climber constants
    private final double RAMP_RATE = 0.2; // seconds

    // State variables
    private boolean climberInverted;

    /** Constructs new Climber object and configures devices */
    public Climber() {
        // Slave control
        //leftSlave.follow(climberLeft);
        //rightSlave.follow(climberRight);
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
        climberInverted = false;
        setInverted(false);
    }

    /** Sets climber motors to given percentage (-1.0, 1.0) */
    public void set(double percent) {
        climberLeft.set(0.5);
        climberRight.set(0.5);

    }

    /** Inverts the the climber. */
    public void setInverted(boolean isInverted) {
        if(climberInverted) isInverted = !isInverted;
        climberLeft.setInverted(isInverted);
        climberRight.setInverted(isInverted);
        //leftSlave.setInverted(isInverted);
        //rightSlave.setInverted(isInverted);
    }

    /** Sets Talons to brake mode - Only mode that should be used. */
    public void setBrake() {
        climberLeft.setNeutralMode(NeutralMode.Brake);
        climberRight.setNeutralMode(NeutralMode.Brake);
        //leftSlave.setNeutralMode(NeutralMode.Brake);
        //rightSlave.setNeutralMode(NeutralMode.Brake);
    }

    /** Returns climber motor output as a percentage. */
    public double getLeftOutput() {
        return climberLeft.get();
    }
    public double getRightOutput() {
        return climberRight.get();
    }

    /** Returns climber motor output current. */
    public double getLeftCurrent() {
        return climberLeft.getOutputCurrent();
    }
    public double getRightCurrent() {
        return climberRight.getOutputCurrent();
    }

    /** Enables ramp rate. */
    public void enableRampRate() {
        climberLeft.configClosedloopRamp(RAMP_RATE);
        climberRight.configClosedloopRamp(RAMP_RATE);
        //rightSlave.configClosedloopRamp(RAMP_RATE);
        //leftSlave.configClosedloopRamp(RAMP_RATE);

    }

}
