package robot.commands.vision;

import robot.Robot;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.Command;

public class FlipLimelight extends Command {

    LimeLight limelight = Robot.limelight;

    public FlipLimelight() {
        requires(limelight);
    }

    @Override
    protected void initialize() {
        limelight.flipCamera();
    }

    @Override
    protected void execute() {
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
    }

}