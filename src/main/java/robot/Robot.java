package robot;

import robot.OI;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    public static Drivetrain drivetrain;
    public static LimeLight limelight;
    public static OI oi;

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        drivetrain = new Drivetrain();
        limelight = new LimeLight();
        oi = new OI();
    }

    @Override
    public void robotPeriodic() {
        limelight.trackRocketBayClose();
        SmartDashboard.putNumber("Gyro", drivetrain.getGyroAngle());
        SmartDashboard.putNumber("limelightAngle", limelight.getHorizontalAngle());
        SmartDashboard.putNumber("limelightDistance", limelight.getDistance(limelight.getPipeline().getWidth()));
        SmartDashboard.putNumber("limelightCorrection", limelight.getCorrectionAngle());
        drivetrain.trackFieldPosition();
        SmartDashboard.putNumber("Field pos x", drivetrain.getFieldPosX());
        SmartDashboard.putNumber("Field pos y", drivetrain.getFieldPosY());
    }

    @Override
    public void disabledInit() {
        drivetrain.resetEncoders();
        drivetrain.resetGyro();
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = chooser.getSelected();
        if (autonomousCommand != null) {
            autonomousCommand.start();
        }
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() {
    }

}
