package robot.subsystems;

import robot.commands.drive.ManualDrive;
import robot.utils.CSPMath;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem {

    // Device initialization
    private CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax leftSlave1 = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax leftSlave2 = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(4, MotorType.kBrushless);
    private CANSparkMax rightSlave1 = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax rightSlave2 = new CANSparkMax(6, MotorType.kBrushless);
    private CANEncoder leftEncoder = new CANEncoder(leftMotor);
    private CANEncoder rightEncoder = new CANEncoder(rightMotor);
    private CANPIDController leftPidC = leftMotor.getPIDController();
    private CANPIDController rightPidC = rightMotor.getPIDController();
    private AHRS ahrs = new AHRS(SerialPort.Port.kMXP);
    private DigitalInput lineSensorLeft = new DigitalInput(0); // yellow wire up
    private DigitalInput lineSensorMid = new DigitalInput(1);
    private DigitalInput lineSensorRight = new DigitalInput(2);
    private DoubleSolenoid gearShift = new DoubleSolenoid(0, 1);

    // Constants
    public final double MAX_VELOCITY = 9; // ft/s
    public final double MAX_ACCELERATION = 5; // ft/s^2
    public final double MAX_JERK = 190; // ft/s^3
    public final double kP = 5e-5;
    public final double kI = 1e-6;
    public final double kD = 0;
    public final double kF = 0;
    public final double kI_ZONE = 0;
    public final int    SLOT_ID = 0;
    public final double MAX_OUT = 1.0;
    public final double WHEELBASE_WIDTH = 2; // ft
    public final double WHEEL_DIAMETER = (6.0 / 12.0); // ft
    public final double TICKS_PER_REV = 1.0; // neo
    public final double LOW_GEAR_RATIO = 15.32;
    public final double HIGH_GEAR_RATIO = 7.08;
    private double      currentGearRatio = HIGH_GEAR_RATIO;
    public final double RAMP_RATE = 0.75; // seconds
    public double       ENCODER_TO_FEET = (WHEEL_DIAMETER * Math.PI) / (TICKS_PER_REV * currentGearRatio); // ft
    public final double DELTA_T = 0.02; // seconds
    public static double brownoutVariable;


    // State vars
    private double fieldPosX, fieldPosY = 0;
    private boolean leftInverted, rightInverted;

    /** Constructs new Drivetrain object and configures devices. */
    public Drivetrain() {

        // Slave control
        leftSlave1.follow(leftMotor);
        leftSlave2.follow(leftMotor);
        rightSlave1.follow(rightMotor);
        rightSlave2.follow(rightMotor);

        // Reset
        controllerInit();
        reset();
        calibrateGyro();

    }

    /** Defines default command that will run when object is created. */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualDrive());
    }

    /** Prints necessary info to dashboard. */
    private void updateShuffleboard() {
        SmartDashboard.putNumber("L Pos", getLeftPosition());
        SmartDashboard.putNumber("R Pos", getRightPosition());
        SmartDashboard.putNumber("L Vel", getLeftVelocity());
        SmartDashboard.putNumber("R Vel", getRightVelocity());
        SmartDashboard.putNumber("Gyro", getGyroAngle());
        SmartDashboard.putNumber("Field X", getFieldPosX());
        SmartDashboard.putNumber("Field Y", getFieldPosY());
        SmartDashboard.putNumber("Target angle", getTargetAngle());
        SmartDashboard.putNumber("L1 temp", leftMotor.getMotorTemperature());
        SmartDashboard.putNumber("L2 temp", leftSlave1.getMotorTemperature());
        SmartDashboard.putNumber("L3 temp", leftSlave2.getMotorTemperature());
        SmartDashboard.putNumber("R4 temp", rightMotor.getMotorTemperature());
        SmartDashboard.putNumber("R5 temp", rightSlave1.getMotorTemperature());
        SmartDashboard.putNumber("R6 temp", rightSlave2.getMotorTemperature());
    }

    /** Runs every loop. */
    @Override
    public void periodic() {
        trackFieldPosition();
        updateShuffleboard();
    }

    /** Resets necessary drive devices. */
    public void reset() {
        resetEncoders();
        resetGyro();
        resetFieldPos();
        enableRampRate();
        setBrake();
        leftInverted = true;
        rightInverted = false;
        setInverted(false);
    }

    /** Configures gains for Spark closed loop controller. */
    private void controllerInit() {
        leftPidC.setP(kP);
        leftPidC.setI(kI);
        leftPidC.setD(kD);
        leftPidC.setIZone(kI_ZONE);
        leftPidC.setFF(kF);
        leftPidC.setOutputRange(-MAX_OUT, MAX_OUT);
        leftPidC.setSmartMotionMaxVelocity(MAX_VELOCITY, SLOT_ID);
        leftPidC.setSmartMotionMaxAccel(MAX_ACCELERATION, SLOT_ID);
        rightPidC.setP(kP);
        rightPidC.setI(kI);
        rightPidC.setD(kD);
        rightPidC.setIZone(kI_ZONE);
        rightPidC.setFF(kF);
        rightPidC.setOutputRange(-MAX_OUT, MAX_OUT);
        rightPidC.setSmartMotionMaxVelocity(MAX_VELOCITY, SLOT_ID);
        rightPidC.setSmartMotionMaxAccel(MAX_ACCELERATION, SLOT_ID);
    }

    /** Sets left motors to given percentage (-1.0 - 1.0). */
    public void setLeft(double percent) {
        leftMotor.set(percent);
    }

    /** Sets right motors to given percentage (-1.0 - 1.0). */
    public void setRight(double percent) {
        rightMotor.set(percent);
    }

    /** Inverts drivetrain. True inverts each side from the
     *  current state set in the Drivetrain constructor. */
    public void setInverted(boolean isInverted) {
        setLeftInverted(isInverted);
        setRightInverted(isInverted);
    }

    /** Inverts the left side of the drivetrain. True inverts it
     *  from the current state set in the Drivetrain constructor. */
    public void setLeftInverted(boolean isInverted) {
        if(leftInverted) isInverted = !isInverted;
        leftMotor.setInverted(isInverted);
        leftSlave1.setInverted(isInverted);
        leftSlave2.setInverted(isInverted);
    }

    /** Inverts the right side of the drivetrain. True inverts it
     *  from the current state set in the Drivetrain constructor. */
    public void setRightInverted(boolean isInverted) {
        if(rightInverted) isInverted = !isInverted;
        rightMotor.setInverted(isInverted);
        rightSlave1.setInverted(isInverted);
        rightSlave2.setInverted(isInverted);
    }

    /** Sets drivetrain to brake mode. */
    public void setBrake() {
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftSlave1.setIdleMode(IdleMode.kBrake);
        leftSlave2.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightSlave1.setIdleMode(IdleMode.kBrake);
        rightSlave2.setIdleMode(IdleMode.kBrake);
    }

    /** Sets drivetrain to coast mode. */
    public void setCoast() {
        leftMotor.setIdleMode(IdleMode.kCoast);
        leftSlave1.setIdleMode(IdleMode.kCoast);
        leftSlave2.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);
        rightSlave1.setIdleMode(IdleMode.kCoast);
        rightSlave2.setIdleMode(IdleMode.kCoast);
    }

    /** Resets encoder values to 0 for both sides of drivetrain. */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /** Returns left encoder position in feet. */
    public double getLeftPosition() {
        return leftEncoder.getPosition() * ENCODER_TO_FEET;
    }

    /** Returns right encoder position in feet. */
    public double getRightPosition() {
        return rightEncoder.getPosition() * ENCODER_TO_FEET;
    }

    /** Returns encoder position in feet as average of left and right encoders. */
    public double getPosition() {
        return (getLeftPosition() + getRightPosition()) / 2;
    }

    /** Returns left encoder position in native Spark units (revolutions) */
    public double getRawLeftPosition() {
        return leftEncoder.getPosition() / LOW_GEAR_RATIO;
    }

    /** Returns left encoder position in native Spark units (revolutions) */
    public double getRawRightPosition() {
        return rightEncoder.getPosition() / LOW_GEAR_RATIO;
    }

    /** Returns left encoder velocity in feet per second. */
    public double getLeftVelocity() {
        return leftEncoder.getVelocity() * ENCODER_TO_FEET / 60.0; // native is rpm
    }

    /** Returns right encoder velocity in feet per second. */
    public double getRightVelocity() {
        return rightEncoder.getVelocity() * ENCODER_TO_FEET / 60.0; // native is rpm
    }

    /** Returns average robot velocity in feet per second. */
    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2;
    }

    /** Returns the left motor output as a percentage. */
    public double getLeftOutput() {
        return leftMotor.get();
    }

        /** Returns the right motor output as a percentage. */
    public double getRightOutput() {
        return rightMotor.get();
    }

    /** Returns average motor output current. */
    public double getMotorCurrent() {
        return (leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent()) / 2;
    }

    /** Returns gyro angle in degrees. */
    public double getGyroAngle() {
        return ahrs.getYaw();
    }

    /** Returns gyro rate in degrees per sec. */
    public double getGyroRate() {
        return ahrs.getRate();
    }

    /** Resets gyro angle to 0. AVOID CALLING THIS. */
    public void resetGyro() {
        ahrs.reset();
    }

    /** Calibrates the gyro to reduce drifting. Only call when robot is not moving. */
    public void calibrateGyro() {
    }

    /** Returns whether or not left photo sensor is reflecting. */
    public boolean getLeftLineSensor() {
        return lineSensorLeft.get();
    }

    /** Returns whether or not left photo sensor is reflecting. */
    public boolean getMidLineSensor() {
        return lineSensorMid.get();
    }

    /** Returns whether or not left photo sensor is reflecting. */
    public boolean getRightLineSensor() {
        return lineSensorRight.get();
    }

    /** Tracks robot position on field. To be called in robotPeriodic(). */
    public void trackFieldPosition() {
        fieldPosX += getVelocity() * Math.cos(Math.toRadians(getGyroAngle())) * DELTA_T;
        fieldPosY += getVelocity() * Math.sin(Math.toRadians(getGyroAngle())) * DELTA_T;
    }

    /** Returns estimate of robot's x (forward / reverse) location relative
     *  to starting point in feet. */
    public double getFieldPosX() {
        return fieldPosX;
    }

    /** Returns estimate of robot's y (left / right) location relative
     *  to starting point in feet. */
    public double getFieldPosY() {
        return fieldPosY;
    }

    /** Resets field position variables to 0. */
    public void resetFieldPos() {
        fieldPosX = 0;
        fieldPosY = 0;
    }

    /** Estimates target angle based off of field position and gyro angle. */
    public double getTargetAngle() {

        // get robot info
        double y = getFieldPosY();
        double theta = getGyroAngle();

        // vars based on info
        double angleDir = (theta > 0) ? 1 : -1;
        boolean inHab = CSPMath.isBetween(y, -6, 6);

        // estimate angle
        if(inHab && CSPMath.isBetween(theta, -30, 30)) {
            return 0; // front of ship
        } else if(!inHab && CSPMath.isBetween(theta, -30, 30)) {
            return 28.75 * angleDir; // front of rocket
        } else if(CSPMath.isBetween(theta, 31 * angleDir, 130 * angleDir)) {
            return 90 * angleDir; // middle of rocket or side of ship
        } else if(CSPMath.isBetween(theta, 131 * angleDir, 180 * angleDir)) {
            return 151.25 * angleDir; // back of rocket
        } else {
            return 0; // default
        }

    }

    /** Enables ramp rate. */
    public void enableRampRate() {
        leftMotor.setOpenLoopRampRate(RAMP_RATE);
        leftMotor.setClosedLoopRampRate(RAMP_RATE);
        rightMotor.setOpenLoopRampRate(RAMP_RATE);
        rightMotor.setClosedLoopRampRate(RAMP_RATE);
    }

    /** Shifts drivetrain to low gear. */
    public void setLowGear() {
        gearShift.set(Value.kForward);
        currentGearRatio = LOW_GEAR_RATIO;
    }

    /** Shifts drivetrain to high gear. */
    public void setHighGear() {
        gearShift.set(Value.kReverse);
        currentGearRatio = LOW_GEAR_RATIO;
    }

    /** Turns gear shift solenoid off. */
    public void setGearShiftOff() {
        gearShift.set(Value.kOff);
    }

    /** Controls drivetrain with arcade model, with positive xSpeed going forward
     *  and positive zTurn turning right. */
    public void arcade(double xSpeed, double zTurn) {

        final double kQUICKSTOP = 1.5;
        final double MAX_INPUT = 1.0;
        double turnRatio;
        double quickStop = 0;

        quickStop += -kQUICKSTOP * zTurn * DELTA_T;
        if(zTurn == 0) zTurn = quickStop;

        double leftInput = xSpeed + zTurn;
        double rightInput = xSpeed - zTurn;

        // ensure that input does not exceed 1.0
        // if it does, reduce greatest input to 1.0 and reduce other proportionately
        if(leftInput > MAX_INPUT || rightInput > MAX_INPUT) {
            if(rightInput > leftInput) {
                turnRatio = leftInput / rightInput;
                rightInput = MAX_INPUT;
                leftInput = MAX_INPUT * turnRatio;
            } else if (leftInput > rightInput) {
                turnRatio = rightInput / leftInput;
                leftInput = MAX_INPUT;
                rightInput = MAX_INPUT * turnRatio;
            }
        } else if(leftInput < -MAX_INPUT || rightInput < -MAX_INPUT) {
            if(rightInput < leftInput) {
                turnRatio = leftInput / rightInput;
                rightInput = -MAX_INPUT;
                leftInput = -MAX_INPUT * turnRatio;
            } else if (leftInput < rightInput) {
                turnRatio = rightInput / leftInput;
                leftInput = -MAX_INPUT;
                rightInput = -MAX_INPUT * turnRatio;
            }
        }

        // command motor output
        setLeft(leftInput);
        setRight(rightInput);

    }

    /** Controls drivetrain with tank model, individually moving left and
     *  right sides. Output multiplied by throttle. */
    public void tank(double leftSpeed, double rightSpeed, double throttle) {
        setLeft(leftSpeed * throttle);
        setRight(rightSpeed * throttle);
    }

    /** Returns temperature of motor based off CAN ID. */
    public double getMotorTemperature(int index){
        CANSparkMax[] sparks = new CANSparkMax[]{
            leftMotor,
            leftSlave1,
            leftSlave2,
            rightMotor,
            rightSlave1,
            rightSlave2
        };
        index -= 1;
        double temp = -1.0;
        try {
            temp = sparks[index].getMotorTemperature();
        } catch(ArrayIndexOutOfBoundsException e) {
            System.err.println("Error: index not in array of drive sparks.");
        }
        return temp;
    }

}
