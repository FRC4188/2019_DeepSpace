package robot.commands.arm;

import robot.Robot;
import robot.subsystems.Arm;
import edu.wpi.first.wpilibj.command.Command;

/** Holds arm angle at current position. */
public class HoldArmAngle extends Command {

    Arm arm = Robot.arm;

    public HoldArmAngle() {
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        arm.holdPosition();
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
