package robot;

import robot.OI;
import robot.subsystems.*;
import robot.utils.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot {

    public static Drivetrain drivetrain;
    public static Arm arm;
    public static Intake intake;
    public static Elevator elevator;
    public static Climber climber;
    public static LimeLight limelight;
    public static LED led;
    public static Logger logger;
    public static BrownoutProtection brownoutProtection;
    public static TemperatureManager tempManager;
    public static AutoChooser autoChooser;
    public static OI oi;

    Command autonomousCommand;

    @Override
    public void robotInit() {

        // initialize subsystems
        //logger = new Logger();
        drivetrain = new Drivetrain();
        arm = new Arm();
        intake = new Intake();
        elevator = new Elevator();
        climber = new Climber();
        limelight = new LimeLight();
        led = new LED();
        brownoutProtection = new BrownoutProtection();
        tempManager = new TemperatureManager();
        autoChooser = new AutoChooser();
        oi = new OI();

        // start camera stream
        CameraServer.getInstance().startAutomaticCapture();

        // finish initialization of BadLog
        //logger.finishInit();

    }

    @Override
    public void robotPeriodic() {
        // services
        //logger.update();
        brownoutProtection.run();
        tempManager.run();
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
        autonomousCommand = autoChooser.getSelectedCommand();
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
