package robot.commands.drive;

import robot.OI;
import robot.Robot;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/** Manually controls drivetrain using pilot controller. */
public class ManualDrive extends Command {

    OI oi = Robot.oi;
    Drivetrain drivetrain = Robot.drivetrain;

    public ManualDrive() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        drivetrain.arcade(oi.getPilotY(Hand.kLeft), oi.getPilotX(Hand.kRight), 1.0);
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
