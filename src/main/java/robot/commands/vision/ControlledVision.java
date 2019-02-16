package robot.commands.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.LimeLight;
import robot.Robot;
import robot.OI.Controller;

public class ControlledVision extends Command {

    public LimeLight limelight;
    NetworkTable table = null;

    private boolean trackingBay = false;

    public ControlledVision() {
        requires(Robot.limelight);
        limelight = Robot.limelight;
        table = NetworkTableInstance.getDefault().getTable("limelightData");
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        if(Robot.oi.getPilotButton(Controller.B)){
            limelight.trackCargo();
            trackingBay = false;
        }
        if(Robot.oi.getPilotButton(Controller.Y)){
            limelight.trackHatch();
            trackingBay = false;
        }

        double distance = limelight.getDistance(limelight.getPipeline().getHeight());
        double relangle = limelight.getHorizontalAngle();
        table.getEntry("distance").setDouble(distance);
        table.getEntry("relangle").setDouble(relangle);
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
