package robot;

import badlog.lib.BadLog;
import robot.OI;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import robot.utils.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {

    public static Drivetrain drivetrain;
    public static LimeLight limelight;
    public static OI oi;
    public static Logger logger;
    

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        logger.init();

        drivetrain = new Drivetrain();
        limelight = new LimeLight();
        oi = new OI();

        BadLog.createTopic("Match Time", "s", () -> DriverStation.getInstance().getMatchTime());
        BadLog.createTopic("Voltage", "V", () -> RobotController.getBatteryVoltage());

        logger.finishInit();
    }

    @Override
    public void robotPeriodic() {
        logger.update();
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
