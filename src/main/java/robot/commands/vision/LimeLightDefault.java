package robot.commands.vision;

import robot.Robot;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.Command;

public class LimeLightDefault extends Command {

    LimeLight limelight = Robot.limelight;

    public LimeLightDefault() {
        requires(limelight);
    }

    @Override
    protected void initialize() {
        limelight.useAsCamera();
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
        limelight.trackBay();
    }

}
