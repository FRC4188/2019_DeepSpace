package robot.subsystems; 

import robot.RobotMap;
import robot.commands.ManualDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Drivetrain extends Subsystem {
   
    TalonSRX left = new TalonSRX(RobotMap.LEFT);
    TalonSRX leftSlave1 = new TalonSRX(RobotMap.LEFT_SLAVE1);
    TalonSRX leftSlave2 = new TalonSRX(RobotMap.LEFT_SLAVE2);
    TalonSRX right = new TalonSRX(RobotMap.RIGHT);
    TalonSRX rightSlave1 = new TalonSRX(RobotMap.RIGHT_SLAVE1);
    TalonSRX rightSlave2 = new TalonSRX(RobotMap.RIGHT_SLAVE2);
   
    /** Constructs new Drivetrain object and configures devices */
    public Drivetrain() {

        // Slave control
        leftSlave1.follow(left);
        leftSlave2.follow(left);
        rightSlave1.follow(right);
        rightSlave2.follow(right);

        // Encoders
        left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        resetEncoders();

        // Enable ramping
        enableRampRate();

    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualDrive());
    }

    /** Resets encoder values to 0 for both sides of drivetrain. */
    public void resetEncoders() {
        left.setSelectedSensorPosition(0, 0, 0);
        right.setSelectedSensorPosition(0, 0, 0);
    }

    /** Enables open and closed loop ramp rate */
    public void enableRampRate() {
        left.configClosedloopRamp(RobotMap.RAMP_RATE);
        left.configOpenloopRamp(RobotMap.RAMP_RATE);
        right.configOpenloopRamp(RobotMap.RAMP_RATE);
        right.configClosedloopRamp(RobotMap.RAMP_RATE);
    }
    
    /** Controls drivetrain with arcade model, with positive xSpeed going forward
     *  and positive zTurn turning right. Output multiplied by throttle. */
    public void arcade(double xSpeed, double zTurn, double throttle) {
        left.set(ControlMode.PercentOutput, (xSpeed + zTurn) * throttle);
        right.set(ControlMode.PercentOutput, (xSpeed + zTurn) * throttle);
    }

    /** Controls drivetrain with tank model, individually moving left and
     *  right sides. Output multiplied by throttle. */
    public void tank(double leftSpeed, double rightSpeed, double throttle) {
        left.set(ControlMode.PercentOutput, leftSpeed * throttle);
        right.set(ControlMode.PercentOutput, rightSpeed * throttle);
    }

}
