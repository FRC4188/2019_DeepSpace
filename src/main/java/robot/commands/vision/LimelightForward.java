package robot.commands.vision;

import robot.Robot;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.Command;

/** Points the Limelight servo forward. */
public class LimelightForward extends Command {

    LimeLight limelight = Robot.limelight;

    public LimelightForward() {
    }

    @Override
    protected void initialize() {
        limelight.lookForward();
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