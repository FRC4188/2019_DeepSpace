package robot.commands.climb;

import robot.Robot;
import robot.subsystems.Arm;
import robot.subsystems.Climber;
import edu.wpi.first.wpilibj.command.Command;

/** Runs climber motors at given percent, ignoring limits.
 *  Positive percent extends. */
public class ClimbWithoutSwitches extends Command {

    Climber climber = Robot.climber;
    Arm arm = Robot.arm;
    double percent;

    public ClimbWithoutSwitches(double percent) {
        this.percent = percent;
        requires(arm);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        arm.setOpenLoop(0.39, false);
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
