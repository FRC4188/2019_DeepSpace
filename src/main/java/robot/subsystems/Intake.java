package robot.subsystems;

import robot.commands.intake.ManualWrist;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import badlog.lib.BadLog;
import robot.utils.Logger;

public class Intake extends Subsystem {

    // Device initialization
    private WPI_TalonSRX wristMotor = new WPI_TalonSRX(31);
    private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(32);
    private DoubleSolenoid hatchSolenoid = new DoubleSolenoid(2, 3);

    // Constants
    private final double INITIAL_ANGLE = 0; // degrees
    private final double ENCODER_TO_DEGREES = 360.0 / 368870.0; // degrees
    private final double RAMP_RATE = 0.2; // seconds
    private final double MAX_OUT = 0.5; // percent out
    private final double MAX_VELOCITY = 20000.0; // talon units per 100ms
    private final int    CRUISE_VEL = 15000; // talon units per 100ms
    private final int    CRUISE_ACCEL = 15000; // talon units per 100ms per sec
    private final double kP = 0.025;
    private final double kI = 0;
    private final double kD = 0.0;
    private final double kF = 1023 / MAX_VELOCITY;
    private final int    SLOT_ID = 0;
    private final int    TIMEOUT = 10; // ms

    // State enums
    public enum WristState { CARGO, HATCH }
    public enum IntakeState { EMPTY, FULL }

    // State vars
    private boolean intakeInverted, wristInverted;
    private WristState wristState;
    private IntakeState intakeState;
    private int hatchState = -1;

    /** Constructs intake object and configures devices. */
    public Intake(){

        // Encoders
        wristMotor.configSelectedFeedbackSensor(
                FeedbackDevice.CTRE_MagEncoder_Relative, SLOT_ID, TIMEOUT);
        wristMotor.setSensorPhase(true);

        // Reset
        controllerInit();
        reset();

        // Initialize BadLog
        //initializeBadLog();
    }

    @Override
    /** Default command, runs when object is created. */
    protected void initDefaultCommand() {
        setDefaultCommand(new ManualWrist());
    }

    /** Prints info to dashboard. */
    private void updateShufleboard() {
        SmartDashboard.putNumber("Wrist pos", getWristPosition());
        SmartDashboard.putNumber("Wrist raw pos", getRawWristPosition());
        SmartDashboard.putNumber("Wrist raw vel", getRawWristVelocity());
        SmartDashboard.putNumber("Hatch", getHatchState());
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
        intakeInverted = false;
        wristInverted = false;
        setInverted(false);
        setWristState(WristState.HATCH);
        setIntakeState(IntakeState.FULL);
    }

    /** Configures gains for SRX closed loop controller. */
    private void controllerInit() {
        wristMotor.configNominalOutputForward(0, TIMEOUT);
        wristMotor.configNominalOutputReverse(0, TIMEOUT);
        wristMotor.configPeakOutputForward(MAX_OUT, TIMEOUT);
        wristMotor.configPeakOutputReverse(-MAX_OUT, TIMEOUT);
        wristMotor.config_kP(SLOT_ID, kP, TIMEOUT);
        wristMotor.config_kI(SLOT_ID, kI, TIMEOUT);
        wristMotor.config_kD(SLOT_ID, kD, TIMEOUT);
        wristMotor.config_kF(SLOT_ID, kF, TIMEOUT);
        wristMotor.configMotionCruiseVelocity(CRUISE_VEL, TIMEOUT);
        wristMotor.configMotionAcceleration(CRUISE_ACCEL, TIMEOUT);
    }

    /** Creates topics for BadLog. */
    public void initializeBadLog() {
        BadLog.createTopic("Intake/Wrist Position", "degs", () -> getWristPosition());
        BadLog.createTopic("Intake/Wrist Velocity", "deg/s", () -> getWristVelocity());
        BadLog.createTopic("Intake/Wrist Current", "amps", () -> getWristCurrent());
        BadLog.createTopic("Intake/Current", "amps", () -> getIntakeCurrent());
   }

    /** Sets intake motors to given percentage (-1.0, 1.0) */
    public void spinIntake(double percent) {
        intakeMotor.set(ControlMode.PercentOutput, percent);
    }

    /** Sets intake wrist motors to given percentage (-1.0, 1.0) using velocity controller. */
    public void setWrist(double percent) {
        double output = percent * CRUISE_VEL;
        wristMotor.set(ControlMode.Velocity, output);
    }

    /** Sets intake wrist motors to given percentage (-1.0, 1.0) */
    public void setWristOpenLoop(double percent) {
        wristMotor.set(ControlMode.PercentOutput, percent);
    }


    /** Sets wrist to given angle in degrees. */
    public void wristToAngle(double angleInDegrees, double toleranceInDegrees) {
        double targetPos = angleInDegrees / ENCODER_TO_DEGREES;
        int tolerance = (int) (toleranceInDegrees / ENCODER_TO_DEGREES);
        wristMotor.configAllowableClosedloopError(SLOT_ID, tolerance, TIMEOUT);
        wristMotor.set(ControlMode.MotionMagic, targetPos);
    }

    /** Fires hatch cylinders inward. */
    public void hatchCylindersIn() {
        setHatchState(-1);
        hatchSolenoid.set(Value.kReverse);
    }

    /** Fires hatch cylinders outward. */
    public void hatchCylindersOut() {
        setHatchState(1);
        hatchSolenoid.set(Value.kForward);
    }

    public void hatchSolenoidOff() {
        hatchSolenoid.set(Value.kOff);
    }

    public void setHatchSolenoid(Value value) {
        if (value == Value.kReverse){
            setHatchState(1);
        }
        else if (value == Value.kForward){
            setHatchState(-1);
        }
        hatchSolenoid.set(value);
    }

    /** Inverts intake. */
    public void setInverted(boolean isInverted) {
        setIntakeInverted(isInverted);
        setWristInverted(isInverted);
    }

    /** Inverts the motors of the intake. */
    public void setIntakeInverted(boolean isInverted) {
        if(intakeInverted) isInverted = !isInverted;
        intakeMotor.setInverted(isInverted);
    }

    /** Inverts the wrist of the intake. */
    public void setWristInverted(boolean isInverted) {
        if(wristInverted) isInverted = !isInverted;
        wristMotor.setInverted(isInverted);
    }

    /** Sets intake talons to brake mode. */
    public void setBrake() {
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setNeutralMode(NeutralMode.Brake);
    }

    /** Sets intake talons to coast mode. */
    public void setCoast() {
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        wristMotor.setNeutralMode(NeutralMode.Coast);
    }

    /** Resets encoder values to 0 for both intake and intake wrist. */
    public void resetEncoders() {
        int init = (int) (INITIAL_ANGLE / ENCODER_TO_DEGREES);
        wristMotor.setSelectedSensorPosition(init, 0, 10);
    }

    /** Sets encoder to degree value wristAngle. */
    public void setEncoders(double wristAngle) {
        double ang = wristAngle/ENCODER_TO_DEGREES;
        wristMotor.setSelectedSensorPosition((int) ang, 0, 10);
    }

    /** Returns intake wrist encoder position in degrees. */
    public double getWristPosition() {
        return wristMotor.getSelectedSensorPosition(0) * ENCODER_TO_DEGREES;
    }

    /** Returns intake wrist encoder position in native talon units. */
    public double getRawWristPosition() {
        return wristMotor.getSelectedSensorPosition();
    }

    /** Returns intake wrist encoder velocity in degrees per second. */
    public double getWristVelocity() {
        return wristMotor.getSelectedSensorVelocity() * ENCODER_TO_DEGREES * 10; // native talon is per 100ms
    }

    /** Returns intake wrist encoder velocity in native talon units. */
    public double getRawWristVelocity() {
        return wristMotor.getSelectedSensorVelocity(0);
    }

    /** Returns intake motor output as a percentage. */
    public double getIntakeOutput() {
        return intakeMotor.getMotorOutputPercent();
    }

    /** Returns intake wrist motor output as a percentage. */
    public double getWristOutput() {
        return wristMotor.getMotorOutputPercent();
    }

    /** Returns intake motor output current. */
    public double getIntakeCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    /** Returns intake wrist motor output current. */
    public double getWristCurrent() {
        return wristMotor.getOutputCurrent();
    }

    /** Enables open and closed loop ramp rate. */
    public void enableRampRate() {
        intakeMotor.configClosedloopRamp(RAMP_RATE);
        intakeMotor.configOpenloopRamp(RAMP_RATE);
        wristMotor.configOpenloopRamp(RAMP_RATE);
        wristMotor.configClosedloopRamp(RAMP_RATE);
    }

    /** Returns the current state of the wrist orientation (cargo or hatch). */
    public WristState getWristState() {
        return wristState;
    }

    /** Returns the current state of the intake (empty or full). */
    public IntakeState getIntakeState() {
        return intakeState;
    }

    /** Sets wrist orientation state variable to cargo or hatch. */
    public void setWristState(WristState state) {
        wristState = state;
    }

    /** Sets intake state variable to empty or full. */
    public void setIntakeState(IntakeState state) {
        intakeState = state;
    }

    /** Sets the hatchState int to in(1) or out(2). */
    public void setHatchState(int state){
        hatchState = state;
    }
    
    public int getHatchState(){
        return hatchState;
    }
}
