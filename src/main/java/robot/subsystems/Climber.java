package robot.subsystems;

import robot.commands.climb.ManualClimb;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import badlog.lib.BadLog;

public class Climber extends Subsystem {

    // Device initialization
    private WPI_TalonSRX leftClimberMotor = new WPI_TalonSRX(41);
    private WPI_TalonSRX rightClimberMotor= new WPI_TalonSRX(42);
    private DigitalInput leftTopSwitch = new DigitalInput(5);
    private DigitalInput rightTopSwitch = new DigitalInput(6);
    private DigitalInput leftBottomSwitch = new DigitalInput(7);
    private DigitalInput rightBottomSwitch = new DigitalInput(8);

    // Constants
    private final double RAMP_RATE = 0.2; // seconds
    private final double MAX_OUT = 1.0; // percent out
    private final double MAX_VELOCITY = 6000.0; // talon units per 100ms
    private final double kP = 0.01;
    private final double kI = 0;
    private final double kD = 0;
    private final double kF = 1023 / MAX_VELOCITY;
    private final int    SLOT_ID = 0;
    private final int    TIMEOUT = 10; // ms

    // State variables
    private boolean climberInverted;

    /** Constructs new Climber object and configures devices */
    public Climber() {

        // Encoders
        leftClimberMotor.configSelectedFeedbackSensor(
                FeedbackDevice.CTRE_MagEncoder_Relative, SLOT_ID, TIMEOUT);
        rightClimberMotor.configSelectedFeedbackSensor(
                FeedbackDevice.CTRE_MagEncoder_Relative, SLOT_ID, TIMEOUT);
        leftClimberMotor.setSensorPhase(true);
        rightClimberMotor.setSensorPhase(true);

        // Reset
        controllerInit();
        reset();

        // Initialize Badlog
        initializeBadLog();

    }

    /** Defines default command that will run when object is created. */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualClimb(0));
    }

    /** Prints necessary info to the dashboard. */
    private void updateShufleboard() {
        SmartDashboard.putBoolean("Climber left top switch", getLeftTopSwitch());
        SmartDashboard.putBoolean("Climber right top switch", getRightTopSwitch());
        SmartDashboard.putBoolean("Climber left bottom switch", getLeftBottomSwitch());
        SmartDashboard.putBoolean("Climber right bottom switch", getRightBottomSwitch());
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
        climberInverted = true;
        setInverted(false);
    }

    /** Configures gains for SRX closed loop controller. */
    private void controllerInit() {
        leftClimberMotor.configNominalOutputForward(0, TIMEOUT);
        leftClimberMotor.configNominalOutputReverse(0, TIMEOUT);
        leftClimberMotor.configPeakOutputForward(MAX_OUT, TIMEOUT);
        leftClimberMotor.configPeakOutputReverse(-MAX_OUT, TIMEOUT);
        leftClimberMotor.config_kP(SLOT_ID, kP, TIMEOUT);
        leftClimberMotor.config_kI(SLOT_ID, kI, TIMEOUT);
        leftClimberMotor.config_kD(SLOT_ID, kD, TIMEOUT);
        leftClimberMotor.config_kF(SLOT_ID, kF, TIMEOUT);
        rightClimberMotor.configNominalOutputForward(0, TIMEOUT);
        rightClimberMotor.configNominalOutputReverse(0, TIMEOUT);
        rightClimberMotor.configPeakOutputForward(MAX_OUT, TIMEOUT);
        rightClimberMotor.configPeakOutputReverse(-MAX_OUT, TIMEOUT);
        rightClimberMotor.config_kP(SLOT_ID, kP, TIMEOUT);
        rightClimberMotor.config_kI(SLOT_ID, kI, TIMEOUT);
        rightClimberMotor.config_kD(SLOT_ID, kD, TIMEOUT);
        rightClimberMotor.config_kF(SLOT_ID, kF, TIMEOUT);
    }

    /** Sets left climber motor to given percentage (-1.0, 1.0). 
     *  Positive percent extends climbers. */
    public void setLeft(double percent) {
        double output = percent * MAX_VELOCITY;
        leftClimberMotor.set(ControlMode.Velocity, output);
    }

    /** Sets right climber motor to given percentage (-1.0, 1.0) 
     *  of max velocity. Positive percent extends climbers. */
    public void setRight(double percent) {
        double output = percent * MAX_VELOCITY;
        rightClimberMotor.set(ControlMode.Velocity, output);
    }

    /** Sets left climber motor to given percentage (-1.0, 1.0)
     *  of max velocity. Positive percent extends climbers. */
    public void setLeftOpenLoop(double percent) {
        leftClimberMotor.set(percent);
    }

    /** Sets right climber motor to given percentage (-1.0, 1.0). 
     *  Positive percent extends climbers. */
    public void setRightOpenLoop(double percent) {
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

    /** Returns state of top left climber magnetic limit switch.
     *  True means it is on magnet. */
    public boolean getLeftTopSwitch() {
        return !leftTopSwitch.get();
    }

    /** Returns state of top right climber magnetic limit switch.
     *  True means it is on magnet. */
    public boolean getRightTopSwitch() {
        return !rightTopSwitch.get();
    }

    /** Returns state of bottom left climber magnetic limit switch.
     *  True means it is on magnet. */
    public boolean getLeftBottomSwitch() {
        return !leftBottomSwitch.get();
    }

    /** Returns state of bottom right climber magnetic limit switch.
     *  True means it is on magnet. */
    public boolean getRightBottomSwitch() {
        return !rightBottomSwitch.get();
    }

    /** Enables ramp rate. */
    public void enableRampRate() {
        leftClimberMotor.configOpenloopRamp(RAMP_RATE);
        rightClimberMotor.configOpenloopRamp(RAMP_RATE);
        leftClimberMotor.configClosedloopRamp(RAMP_RATE);
        rightClimberMotor.configClosedloopRamp(RAMP_RATE);
    }

    /** Creates topics for BadLog. */
    public void initializeBadLog() {
        // BadLog.createTopic("Left Top Switch", BadLog.UNITLESS, () -> getLeftTopSwitch());
        // BadLog.createTopic("Right Top Switch", BadLog.UNITLESS, () -> getRightTopSwitch());
        // BadLog.createTopic("Left Bottom Switch", BadLog.UNITLESS, () -> getLeftBottomSwitch());
        // ,BadLog.createTopic("Right Bottom Swtich", BadLog.UNITLESS, () -> getRightBottomSwitch());
    }

}
