package robot.commands.vision;

import robot.Robot;
import robot.subsystems.LimeLight;
import robot.subsystems.LimeLight.Pipeline;
import edu.wpi.first.wpilibj.command.Command;

public class TrackPipeline extends Command {

    LimeLight limelight = Robot.limelight;
    Pipeline pl;

    public TrackPipeline(Pipeline pipeline) {
        requires(limelight);
        pl = pipeline;
    }

    @Override
    protected void initialize() {
        switch(pl) {
        case BAY_CLOSE:
            limelight.trackBay();
            break;
        case BAY_3D:
            limelight.trackBay3D();
            break;
        case CARGO:
            limelight.trackCargo();
            break;
        default:
            break;
        }
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
