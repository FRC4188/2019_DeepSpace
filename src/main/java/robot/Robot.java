package robot;

import robot.OI;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    public static Drivetrain m_drivetrain;
    public static OI m_oi;

    Command m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        m_drivetrain = new Drivetrain();
        m_oi = new OI();
    }

    @Override
    public void robotPeriodic() {
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
        m_autonomousCommand = m_chooser.getSelected();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.start();
        }
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_drivetrain.resetEncoders();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        SmartDashboard.putNumber("Left position", m_drivetrain.getLeftPosition());
        SmartDashboard.putNumber("Right position", m_drivetrain.getRightPosition());
        SmartDashboard.putNumber("Raw left", m_drivetrain.getRawRightPosition());
        SmartDashboard.putNumber("Raw right", m_drivetrain.getRawLeftPosition());
    }

    @Override
    public void testPeriodic() {
    }

}
