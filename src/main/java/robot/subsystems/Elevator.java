package robot.subsystems;

import robot.commands.elevator.ManualElevator;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends Subsystem {

    // Device initialization
    private CANSparkMax elevatorMotor = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax elevatorSlave = new CANSparkMax(12, MotorType.kBrushless);
    private CANEncoder elevatorEncoder = new CANEncoder(elevatorMotor);
    private CANPIDController pidC = elevatorMotor.getPIDController();

    // Constants
    private final double TICKS_PER_REV = 1; // neo
    private final double SPOOL_DIAMETER = (2 / 12); //feet
    private final double GEAR_RATIO = 2.84;
    private final double ENCODER_TO_FEET = (SPOOL_DIAMETER * Math.PI) / (TICKS_PER_REV * GEAR_RATIO); // feet
    private final double RAMP_RATE = 0.2; // seconds
    private final double MAX_OUT = 0.5; // percent out
    private final double kP = 5e-5;
    private final double kI = 1e-6;
    private final double kI_ZONE = 0;
    private final double kD = 0;
    private final double kF = 0;
    private final double MAX_VELOCITY = 1500; // rpm
    private final double MAX_ACCELERATION = 1000;
    private final int    SLOT_ID = 0;

    // State vars
    private boolean elevatorInverted;

    /** Constructs new Elevator object and configures devices */
    public Elevator() {

        // Slave control
        elevatorSlave.follow(elevatorMotor, true);

        // Reset
        controllerInit();
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
        SmartDashboard.putNumber("Elevator raw velocity", getRawVelocity());
        SmartDashboard.putNumber("E11 temp", elevatorMotor.getMotorTemperature());
        SmartDashboard.putNumber("E12 temp", elevatorSlave.getMotorTemperature());
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

    /** Configures gains for Spark closed loop controller. */
    private void controllerInit() {
        pidC.setP(kP);
        pidC.setI(kI);
        pidC.setIZone(kI_ZONE);
        pidC.setD(kD);
        pidC.setFF(kF);
        pidC.setOutputRange(-MAX_OUT, MAX_OUT);
        pidC.setSmartMotionMaxVelocity(MAX_VELOCITY, SLOT_ID);
        pidC.setSmartMotionMaxAccel(MAX_ACCELERATION, SLOT_ID);
    }

    /** Sets elevator motors to given percentage using velocity controller. */
    public void set(double percent) {
        double setpoint = percent * MAX_VELOCITY;
        pidC.setReference(setpoint, ControlType.kVelocity);
    }

    /** Sets elevator motors to given percentage (-1.0, 1.0). */
    public void setOpenLoop(double percent) {
        elevatorMotor.set(percent);
    }

    /** Drives shoulder motor to given angle in degrees. */
    public void elevatorToHeight(double height, double tolerance) {
        // convert from degrees to rotations (Spark units)
        height /= ENCODER_TO_FEET;
        tolerance /= ENCODER_TO_FEET;
        pidC.setSmartMotionAllowedClosedLoopError(tolerance, SLOT_ID);
        pidC.setReference(height, ControlType.kSmartMotion);
    }

    /** Inverts the the elevator. */
    public void setInverted(boolean isInverted) {
        if(elevatorInverted) isInverted = !isInverted;
        elevatorMotor.setInverted(isInverted);
    }

    /** Sets sparks to brake mode - Only mode that should be used. */
    public void setBrake() {
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorSlave.setIdleMode(IdleMode.kBrake);
    }

    /** Resets encoder values to 0 for elevator. */
    public void resetEncoders() {
        elevatorEncoder.setPosition(0);
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
        return elevatorEncoder.getVelocity() * ENCODER_TO_FEET / 60.0; // native is rpm
    }

    /** Returns elevator encoder velocity in native spark units (rpm). */
    public double getRawVelocity() {
        return elevatorEncoder.getVelocity();
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
        elevatorMotor.setOpenLoopRampRate(RAMP_RATE);
        elevatorMotor.setClosedLoopRampRate(RAMP_RATE);
    }

    /** Returns temperature of motor based off CAN ID. */
    public double getMotorTemperature(int index) {
        CANSparkMax[] sparks = new CANSparkMax[]{
            elevatorMotor,
            elevatorSlave
        };
        index -= 11;
        double temp = -1.0;
        try {
            temp = sparks[index].getMotorTemperature();
        } catch(ArrayIndexOutOfBoundsException e) {
            System.err.println("Error: index not in array of elevator sparks.");
        }
        return temp;
    }

}
