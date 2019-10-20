package robot.commands.vision;

import robot.Robot;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.Command;

/** Points the Limelight servo backwards. */
public class LimelightBackward extends Command {

    LimeLight limelight = Robot.limelight;

    public LimelightBackward() {
    }

    @Override
    protected void initialize() {
        limelight.lookBackward();
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