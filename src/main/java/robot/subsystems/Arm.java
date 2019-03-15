package robot.subsystems;

import robot.commands.arm.ManualArm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import badlog.lib.BadLog;
import badlog.lib.DataInferMode;

public class Arm extends Subsystem {

    // Device initialization
    public CANSparkMax shoulderMotor = new CANSparkMax(21, MotorType.kBrushless);
    private CANSparkMax shoulderSlave = new CANSparkMax(22, MotorType.kBrushless);
    public CANEncoder shoulderEncoder = new CANEncoder(shoulderMotor);
    public CANPIDController pidC = shoulderMotor.getPIDController();

    // Constants
    private final double INITIAL_ANGLE = 0; // degrees, 0 when arm is parallel with ground at front of bot
    private final double ENCODER_TO_DEGREES = 180.0 / 35.66; // degrees
    private final double RAMP_RATE = 0.5; // seconds
    private final double FLAT_RATE = 0.035; // percent out
    private final double MAX_VELOCITY = 1000; // rpm
    private final double MAX_ACCELERATION = 1500;
    private final double kP = 5e-5;
    private final double kI = 1e-6;
    private final double kD = 0;
    private final double kF = 0;
    private final double kI_ZONE = 0;
    private final int    SLOT_ID = 0;
    public final double  MAX_OUT = 1.0; // percent out

    // State vars
    private boolean shoulderInverted;

    /** Constructs new Arm object and configures devices */
    public Arm() {

        // Slave control
        shoulderSlave.follow(shoulderMotor, true);

        // Reset
        controllerInit();
        reset();

        // Initialize BadLog
        //initializeBadLog();
    }

    /** Defines default command that will run when object is created. */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualArm());
    }

    /** Prints necessary info to the dashboard. */
    private void updateShufleboard() {
        SmartDashboard.putNumber("Shoulder pos", getPosition());
        SmartDashboard.putNumber("Shoulder raw pos", getRawPosition());
        SmartDashboard.putNumber("S21 temp", shoulderMotor.getMotorTemperature());
        SmartDashboard.putNumber("S22 temp", shoulderSlave.getMotorTemperature());
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
        shoulderInverted = true;
        setInverted(false);
    }

    /** Configures gains for Spark closed loop controller. */
    private void controllerInit() {
        pidC.setP(kP);
        pidC.setI(kI);
        pidC.setD(kD);
        pidC.setIZone(kI_ZONE);
        pidC.setFF(kF);
        pidC.setOutputRange(-MAX_OUT, MAX_OUT);
        pidC.setSmartMotionMaxVelocity(MAX_VELOCITY, SLOT_ID);
        pidC.setSmartMotionMaxAccel(MAX_ACCELERATION, SLOT_ID);
        pidC.setSmartMotionMinOutputVelocity(0, SLOT_ID);
    }

    /** Sets shoulder motors to given percentage using velocity controller. */
    public void set(double percent) {
        double setpoint = percent * MAX_VELOCITY;
        pidC.setReference(setpoint, ControlType.kVelocity);
    }

    /** Sets shoulder motors to given percentage (-1.0, 1.0). If flat rate
     *  is true, will apply voltage based on angle to counteract gravity. */
    public void setOpenLoop(double percent, boolean flatRate) {
        double output = (flatRate) ? percent + FLAT_RATE *
                Math.sin(Math.toRadians(getPosition())) : percent;
        shoulderMotor.set(output);
    }

    /** Drives shoulder motor to given angle in degrees (off the positive x axis). */
    public void shoulderToAngle(double angle, double tolerance) {
        // convert from degrees to rotations (Spark units)
        angle /= ENCODER_TO_DEGREES;
        tolerance /= ENCODER_TO_DEGREES;
        pidC.setSmartMotionAllowedClosedLoopError(tolerance, SLOT_ID);
        pidC.setReference(angle, ControlType.kSmartMotion);
    }

    /** Inverts the the arm. */
    public void setInverted(boolean isInverted) {
        if(shoulderInverted) isInverted = !isInverted;
        shoulderMotor.setInverted(isInverted);
    }

    /** Sets shoulder to brake mode - Only mode that should be used. */
    public void setBrake() {
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        shoulderSlave.setIdleMode(IdleMode.kBrake);
    }

    /** Resets shoulder encoder value to 0. */
    public void resetEncoders() {
        double init = INITIAL_ANGLE / ENCODER_TO_DEGREES;
        shoulderEncoder.setPosition(init);
    }

    /** Returns left encoder position in degrees. */
    public double getPosition() {
        return (shoulderEncoder.getPosition() * ENCODER_TO_DEGREES);
    }

    /** Returns shoulder encoder position in native Spark units (revolutions). */
    public double getRawPosition() {
        return shoulderEncoder.getPosition();
    }

    /** Returns shoulder encoder velocity in degrees per second. */
    public double getVelocity() {
        return shoulderEncoder.getVelocity() * ENCODER_TO_DEGREES / 60.0; // native is rpm
    }

    /** Returns shoulder encoder velocity in native Spark units (rpm). */
    public double getRawVelocity() {
        return shoulderEncoder.getVelocity();
    }

    /** Returns shoulder motor output as a percentage. */
    public double getOutput() {
        return shoulderMotor.get();
    }

    /** Returns shoulder motor output current. */
    public double getCurrent() {
        return shoulderMotor.getOutputCurrent();
    }

    /** Enables open and closed loop ramp rate. */
    public void enableRampRate() {
        shoulderMotor.setOpenLoopRampRate(RAMP_RATE);
        shoulderMotor.setClosedLoopRampRate(RAMP_RATE);
    }

    /** Returns temperature (Celsius) of motor based off CAN ID. */
    public double getMotorTemperature(int index) {
        CANSparkMax[] sparks = new CANSparkMax[]{
            shoulderMotor,
            shoulderSlave,
        };
        index -= 21;
        double temp = -1.0;
        try {
            temp = sparks[index].getMotorTemperature();
        } catch(ArrayIndexOutOfBoundsException e) {
            System.err.println("Error: index " + index + " not in array of arm sparks.");
        }
        return temp;
    }


    /** Creates topics for BadLog. */
    public void initializeBadLog() {
        // Create new method
        BadLog.createTopic("Arm/Position", "deg", () -> getPosition());
        BadLog.createTopic("Arm/Velocity", "deg/s", () -> getVelocity());
        BadLog.createTopic("Arm/Current", "amps", () -> getCurrent());
        BadLog.createTopic("21 Temp", "C", () -> shoulderMotor.getMotorTemperature(), "hide", "join:Arm/Temp");
        BadLog.createTopic("22 Temp", "C", () -> shoulderSlave.getMotorTemperature(), "hide", "join:Arm/Temp");
     }
    
}
