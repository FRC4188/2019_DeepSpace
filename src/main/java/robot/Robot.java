package robot;

import robot.OI;
import robot.subsystems.*;
import robot.utils.Logger;
import badlog.lib.BadLog;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {

    public static Drivetrain drivetrain;
    public static Arm arm;
    public static Intake intake;
    public static LimeLight limelight;
    public static Elevator elevator;
    public static OI oi;
    public static Logger logger;

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    @Override
    public void robotInit() {

        // initialize subsystems
        drivetrain = new Drivetrain();
        arm = new Arm();
        intake = new Intake();
        //elevator = new Elevator();
        limelight = new LimeLight();
        oi = new OI();

        // start logger
        //logger.init();

        // camera
        CameraServer.getInstance().startAutomaticCapture();

    }

    @Override
    public void robotPeriodic() {
        //logger.update();
        limelight.trackRocketBayClose();
    }

    @Override
    public void disabledInit() {
        drivetrain.reset();
        arm.reset();
        intake.reset();
        //elevator.reset();
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
