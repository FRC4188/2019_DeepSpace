package robot.commands.arm;

import robot.Robot;
import robot.subsystems.Arm;
import edu.wpi.first.wpilibj.command.Command;

/** Sets shoulder encoders to 0. */
public class ZeroShoulder extends Command {

    Arm arm = Robot.arm;

    public ZeroShoulder() {
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        arm.resetEncoders();
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
        end();
    }

}
