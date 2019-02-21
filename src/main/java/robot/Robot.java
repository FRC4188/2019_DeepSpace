package robot;

import robot.OI;
import robot.subsystems.*;
import robot.utils.*;
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
    public static Climber climber;
    public static LED led;

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    @Override
    public void robotInit() {

        // initialize subsystems
        logger = new Logger();
        drivetrain = new Drivetrain();
        arm = new Arm();
        intake = new Intake();
        elevator = new Elevator();
        limelight = new LimeLight();
        climber = new Climber();
        led = new LED();
        oi = new OI();

        // start camera stream
        CameraServer.getInstance().startAutomaticCapture();

    }

    @Override
    public void robotPeriodic() {
        logger.update();
    }

    @Override
    public void disabledInit() {

        // reset subsystems
        drivetrain.reset();
        arm.reset();
        intake.reset();
        elevator.reset();
        climber.reset();

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
        Brownout.isBrownout();
    }

    @Override
    public void testPeriodic() {
    }

}
