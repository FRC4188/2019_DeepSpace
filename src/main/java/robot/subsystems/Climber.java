package robot.subsystems;

import robot.commands.climb.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import badlog.lib.BadLog;
import robot.utils.Logger;

public class Climber extends Subsystem {

    // Device initialization
    private WPI_TalonSRX leftClimberMotor = new WPI_TalonSRX(41);
    private WPI_TalonSRX rightClimberMotor= new WPI_TalonSRX(42);
    private DigitalInput leftTopSwitch = new DigitalInput(4);
    private DigitalInput rightTopSwitch = new DigitalInput(5);
    private DigitalInput leftBottomSwitch = new DigitalInput(6);
    private DigitalInput rightBottomSwitch = new DigitalInput(7);

    // Constants
    private final double RAMP_RATE = 0.2; // seconds
    private final double MAX_OUT = 1.0; // percent out
    private final double MAX_VELOCITY = 3000.0; // talon units per 100ms
    private final double kP = 0.01;
    private final double kI = 0;
    private final double kD = 0;
    private final double kF = 1023 / MAX_VELOCITY;
    private final int    SLOT_ID = 0;
    private final int    TIMEOUT = 10; // ms
    private final double ENCODER_TO_FEET = 1.0 / 400000; // ft

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

        // Limit Switches
        leftClimberMotor.overrideLimitSwitchesEnable(true);
        leftClimberMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, TIMEOUT);
        leftClimberMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, TIMEOUT);
        rightClimberMotor.overrideLimitSwitchesEnable(true);
        rightClimberMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, TIMEOUT);
        rightClimberMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, TIMEOUT);

        // Reset
        controllerInit();
        reset();

        // Initialize Badlog
        //initializeBadLog();

    }

    /** Defines default command that will run when object is created. */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualClimb(0));
    }

    /** Prints necessary info to the dashboard. */
    private void updateShufleboard() {
        SmartDashboard.putNumber("Climber l vel", getRawLeftVelocity());
        SmartDashboard.putNumber("Climber l pos", getRawLeftPosition());
        SmartDashboard.putNumber("Climber r vel", getRawRightVelocity());
        SmartDashboard.putNumber("Climber r pos", getRawRightPosition());
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

    /** Sets left climber motor to given velocity in feet / sec. */
    public void setLeftVelocity(double percent) {
        double output = percent / ENCODER_TO_FEET;
        leftClimberMotor.set(ControlMode.Velocity, output);
    }

    /** Sets right climber motor to given velocity in feet / sec. */
    public void setRightVelocity(double percent) {
        double output = percent * MAX_VELOCITY;
        rightClimberMotor.set(ControlMode.Velocity, output);
    }

    /** Sets both climbers to given height in feet. */
    public void climberToHeight(double heightInFt, double toleranceInFt) {
        double targetPos = heightInFt / ENCODER_TO_FEET;
        int tolerance = (int) (toleranceInFt / ENCODER_TO_FEET);
        leftClimberMotor.configAllowableClosedloopError(SLOT_ID, tolerance, TIMEOUT);
        leftClimberMotor.set(ControlMode.MotionMagic, targetPos);
        rightClimberMotor.configAllowableClosedloopError(SLOT_ID, tolerance, TIMEOUT);
        rightClimberMotor.set(ControlMode.MotionMagic, targetPos);
    }

    /** Inverts the the climber. */
    public void setInverted(boolean isInverted) {
        if(climberInverted) isInverted = !isInverted;
        leftClimberMotor.setInverted(!isInverted);
        rightClimberMotor.setInverted(isInverted);
    }

    /** Sets Talons to brake mode - Only mode that should be used. */
    public void setBrake() {
        leftClimberMotor.setNeutralMode(NeutralMode.Brake);
        rightClimberMotor.setNeutralMode(NeutralMode.Brake);
    }

    /** Resets encoder values to 0 for both climbers. */
    public void resetEncoders() {
        leftClimberMotor.setSelectedSensorPosition(0);
        rightClimberMotor.setSelectedSensorPosition(0);
    }

    /** Returns left encoder position in feet. */
    public double getLeftPosition() {
        return leftClimberMotor.getSelectedSensorPosition() * ENCODER_TO_FEET;
    }

    /** Returns right encoder position in feet. */
    public double getRightPosition() {
        return rightClimberMotor.getSelectedSensorPosition() * ENCODER_TO_FEET;
    }

    /** Returns encoder position in feet as average of left and right encoders. */
    public double getPosition() {
        return (getLeftPosition() + getRightPosition()) / 2.0;
    }

    /** Returns left encoder position in native talon units. */
    public double getRawLeftPosition() {
        return leftClimberMotor.getSelectedSensorPosition();
    }

    /** Returns left encoder position in native talon units. */
    public double getRawRightPosition() {
        return rightClimberMotor.getSelectedSensorPosition();
    }

    /** Returns left encoder velocity in feet per second. */
    public double getLeftVelocity() {
        return leftClimberMotor.getSelectedSensorVelocity() *
                ENCODER_TO_FEET * 10; // native is per 100ms
    }

    /** Returns right encoder velocity in feet per second. */
    public double getRightVelocity() {
        return rightClimberMotor.getSelectedSensorVelocity() *
                ENCODER_TO_FEET * 10; // native is per 100ms
    }

    /** Returns left encoder velocity in native talon units per 100ms. */
    public double getRawLeftVelocity() {
        return leftClimberMotor.getSelectedSensorVelocity();
    }

    /** Returns right encoder velocity in native talon units per 100ms. */
    public double getRawRightVelocity() {
        return rightClimberMotor.getSelectedSensorVelocity();
    }

    /** Returns average robot velocity in feet per second. */
    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    /** Returns the left motor output as a percentage. */
    public double getLeftOutput() {
        return leftClimberMotor.get();
    }

    /** Returns the right motor output as a percentage. */
    public double getRightOutput() {
        return rightClimberMotor.get();
    }

    /** Returns average motor output current. */
    public double getMotorCurrent() {
        return (leftClimberMotor.getOutputCurrent() +
                rightClimberMotor.getOutputCurrent()) / 2.0;
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
        BadLog.createTopicStr("Left Top Switch", BadLog.UNITLESS, () -> Logger.useBoolean(getLeftTopSwitch()), "hide", "join:Climber/Switches");
        BadLog.createTopicStr("Right Top Switch", BadLog.UNITLESS, () -> Logger.useBoolean(getRightTopSwitch()), "hide", "join:Climber/Switches");
        BadLog.createTopicStr("Left Bottom Switch", BadLog.UNITLESS, () -> Logger.useBoolean(getLeftBottomSwitch()), "hide", "join:Climber/Switches");
        BadLog.createTopicStr("Right Bottom Swtich", BadLog.UNITLESS, () -> Logger.useBoolean(getRightBottomSwitch()), "hide", "join:Climber/Switches");
    }

}
