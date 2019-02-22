package robot.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class LimeLightUseAsCamera extends Command {

    public LimeLightUseAsCamera() {
        requires(Robot.limelight);
    }

    @Override
    protected void initialize() {
        Robot.limelight.useAsCamera();
    }

    @Override
    protected void execute() {
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