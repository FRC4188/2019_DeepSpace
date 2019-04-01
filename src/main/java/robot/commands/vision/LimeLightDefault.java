package robot.commands.vision;

import robot.Robot;
import robot.subsystems.LimeLight;
import robot.subsystems.LimeLight.CameraMode;
import robot.subsystems.LimeLight.Pipeline;
import edu.wpi.first.wpilibj.command.Command;

public class LimeLightDefault extends Command {

    LimeLight limelight = Robot.limelight;

    public LimeLightDefault() {
        requires(limelight);
    }

    @Override
    protected void initialize() {
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
