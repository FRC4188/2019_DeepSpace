package robot.commands.arm;

import robot.OI;
import robot.Robot;
import robot.subsystems.Arm;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class Manipulate extends Command {
        
    OI oi = Robot.oi;
    Arm arm = Robot.arm;

    public Manipulate() {
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
