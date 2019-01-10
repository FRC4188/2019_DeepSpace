package robot.commands.drive;

import robot.Robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class ManualDrive extends Command {

    public static XboxController pilot = Robot.m_oi.pilot;

    public ManualDrive() {
        requires(Robot.m_drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.m_drivetrain.arcade(-pilot.getY(Hand.kLeft), pilot.getX(Hand.kRight), 1.0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }

}
