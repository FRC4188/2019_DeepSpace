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
        SmartDashboard.putNumber("L output", drivetrain.getLeftOutput());
        SmartDashboard.putNumber("L pos", drivetrain.getLeftPosition());
        limelight.trackRocketBayHigh();
        SmartDashboard.putNumber("dist to bay", limelight.getDistance2(15/12));
    }

    @Override
    public void disabledInit() {
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
        drivetrain.resetEncoders();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() {
    }

}
