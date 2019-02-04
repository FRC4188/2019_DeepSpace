package robot;

import robot.OI;
import robot.subsystems.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {

    public static Drivetrain drivetrain;
    public static Arm arm;
    public static LimeLight limelight;
    public static Intake intake;
    public static OI oi;
    public static Elevator elevator;

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        drivetrain = new Drivetrain();
        arm = new Arm();
        intake = new Intake();
        elevator = new Elevator();
        limelight = new LimeLight();
        oi = new OI();
    }

    @Override
    public void robotPeriodic() {
        limelight.trackRocketBayClose();
        drivetrain.trackFieldPosition();
    }

    @Override
    public void disabledInit() {
        drivetrain.reset();
        arm.reset();
        intake.reset();
        elevator.reset();
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
