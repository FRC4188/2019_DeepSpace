package robot.subsystems;

import robot.commands.arm.Manipulate;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends Subsystem {

    // Device initialization
    private TalonSRX shoulder = new TalonSRX(0);
    private TalonSRX shoulderSlave= new TalonSRX(0);

    private TalonSRX wrist = new TalonSRX(0);
    private TalonSRX wristSlave = new TalonSRX(0);

    // Manipulation constants
    private final double TICKS_PER_REV = 4096; // talon units
    private final double ENCODER_TO_RADIANS = (2*Math.PI)/TICKS_PER_REV; // rad
    private final double RAMP_RATE = 0.05; // seconds
    public final double DELTA_T = 0.02; // seconds

    /** Constructs new Arm object and configures devices */
    public Arm() {

        // Slave control
        shoulderSlave.follow(shoulder);
        wristSlave.follow(wrist);

        // Encoders
        shoulder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        resetEncoders();

        // Arm config
        enableRampRate();

        // Set to brake mode
        setBrake();
    }

    /** Defines default command that will run when object is created */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new Manipulate());
    }

    /** Inverts arm. */
    public void setInverted(boolean isInverted) {
        shoulder.setInverted(isInverted);
        shoulderSlave.setInverted(isInverted);
        //leftSlave2.setInverted(isInverted);
        wrist.setInverted(isInverted);
        wristSlave.setInverted(isInverted);
        //rightSlave2.setInverted(isInverted);
    }

    /** Inverts the shoulder of the arm. */
    public void setShoulderInverted(boolean isInverted) {
        shoulder.setInverted(isInverted);
        shoulderSlave.setInverted(isInverted);
        //leftSlave2.setInverted(isInverted);
    }

    /** Inverts the wrist of the arm. */
    public void setWristInverted(boolean isInverted) {
        wrist.setInverted(isInverted);
        wristSlave.setInverted(isInverted);
        //rightSlave2.setInverted(isInverted);
    }

    /** Sets arm talons brake mode - Only mode that should be used. */
    public void setBrake() {
        shoulder.setNeutralMode(NeutralMode.Brake);
        shoulderSlave.setNeutralMode(NeutralMode.Brake);
        //leftSlave2.setNeutralMode(NeutralMode.Brake);
        wrist.setNeutralMode(NeutralMode.Brake);
        wristSlave.setNeutralMode(NeutralMode.Brake);
        //rightSlave2.setNeutralMode(NeutralMode.Brake);
    }

    /** Resets encoder values to 0 for both shoulder and wrist. */
    public void resetEncoders() {
        shoulder.setSelectedSensorPosition(0, 0, 10);
        wrist.setSelectedSensorPosition(0, 0, 10);
    }

    /** Returns left encoder position in radians. */
    public double getShoulderPosition() {
        return shoulder.getSelectedSensorPosition() * ENCODER_TO_RADIANS;
    }

    /** Returns wrist encoder position in feet. */
    public double getWristPosition() {
        return wrist.getSelectedSensorPosition() * ENCODER_TO_RADIANS;
    }

    /** Returns shoulder encoder position in native talon units */
    public double getRawShoulderPosition() {
        return shoulder.getSelectedSensorPosition();
    }

    /** Returns wrist encoder position in native talon units. */
    public double getRawWristtPosition() {
        return wrist.getSelectedSensorPosition();
    }



    /** Enables open and closed loop ramp rate */
    public void enableRampRate() {
        shoulder.configClosedloopRamp(RAMP_RATE);
        shoulder.configOpenloopRamp(RAMP_RATE);
        wrist.configOpenloopRamp(RAMP_RATE);
        wrist.configClosedloopRamp(RAMP_RATE);
    }



    /** Sets arm with given throttle, individually moving shoulder and
     *  wrist. Output multiplied by throttle. */
    public void control(double shoulderSpeed, double wristSpeed, double throttle) {
        shoulder.set(ControlMode.PercentOutput, shoulderSpeed * throttle);
        wrist.set(ControlMode.PercentOutput, wristSpeed * throttle);
    }


}
