package robot.commands.intake;

import robot.OI;
import robot.Robot;
import robot.subsystems.Intake;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class IntakeWrist extends Command {

    OI oi = Robot.oi;
    Intake intake = Robot.intake;

    public IntakeWrist() {
        requires(intake);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        // change this binding, this is already taken by arm
        intake.controlWrist(oi.getCopilotY(Hand.kLeft), 0.5);
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
