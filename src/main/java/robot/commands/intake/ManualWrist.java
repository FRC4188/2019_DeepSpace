package robot.commands.intake;

import robot.OI;
import robot.Robot;
import robot.subsystems.Intake;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/** Moves intake wrist using y val of right copilot stick. */
public class ManualWrist extends Command {

    OI oi = Robot.oi;
    Intake intake = Robot.intake;

    public ManualWrist() {
        requires(intake);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        intake.setWristOpenLoop(oi.getCopilotY(Hand.kRight));
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
