package robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.commands.climb.ManualClimb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber extends Subsystem {

    // Device initialization
    private WPI_TalonSRX leftClimberMotor = new WPI_TalonSRX(41);
    private WPI_TalonSRX rightClimberMotor= new WPI_TalonSRX(42);
    private DigitalInput leftMagnetSwitch = new DigitalInput(5);
    private DigitalInput rightMagnetSwitch = new DigitalInput(6);

    // Constants
    private final double RAMP_RATE = 0.2; // seconds


    // State variables
    private boolean climberInverted;

    /** Constructs new Climber object and configures devices */
    public Climber() {
        reset();
    }

    /** Defines default command that will run when object is created. */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualClimb());
    }

    /** Prints necessary info to the dashboard. */
    private void updateShufleboard() {
        SmartDashboard.putBoolean("Climber left switch", getLeftMagnetSwitch());
        SmartDashboard.putBoolean("Climber right switch", getRightMagnetSwitch());
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

    /** Sets left climber motor to given percentage (-1.0, 1.0). 
     *  Positive percent extends climbers. */
    public void setLeft(double percent) {
        leftClimberMotor.set(percent);
    }

    /** Sets left climber motor to given percentage (-1.0, 1.0). 
     *  Positive percent extends climbers. */
    public void setRight(double percent) {
        rightClimberMotor.set(percent);
    }

    /** Inverts the the climber. */
    public void setInverted(boolean isInverted) {
        if(climberInverted) isInverted = !isInverted;
        leftClimberMotor.setInverted(isInverted);
        rightClimberMotor.setInverted(!isInverted);
    }

    /** Sets Talons to brake mode - Only mode that should be used. */
    public void setBrake() {
        leftClimberMotor.setNeutralMode(NeutralMode.Brake);
        rightClimberMotor.setNeutralMode(NeutralMode.Brake);
    }

    /** Returns state of left climber magnetic limit switch.
     *  True means it is on magnet. */
    public boolean getLeftMagnetSwitch() {
        return !leftMagnetSwitch.get();
    }

    /** Returns state of right climber magnetic limit switch.
     *  True means it is on magnet. */
    public boolean getRightMagnetSwitch() {
        return !rightMagnetSwitch.get();
    }

    /** Enables ramp rate. */
    public void enableRampRate() {
        leftClimberMotor.configOpenloopRamp(RAMP_RATE);
        rightClimberMotor.configOpenloopRamp(RAMP_RATE);
        leftClimberMotor.configClosedloopRamp(RAMP_RATE);
        rightClimberMotor.configClosedloopRamp(RAMP_RATE);
    }

}
