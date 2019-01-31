package robot.commands.arm;

import robot.OI;
import robot.Robot;
import robot.subsystems.Arm;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class ManualArm extends Command {

    OI oi = Robot.oi;
    Arm arm = Robot.arm;

    public ManualArm() {
        requires(arm);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        arm.control(oi.getCopilotY(Hand.kLeft), oi.getCopilotX(Hand.kRight), 1.0);
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
