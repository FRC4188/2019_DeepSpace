/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(Robot.oi.getPilotButton(Controller.A)){
            limelight.trackShipBay();
            trackingBay = true;
        }
        if(Robot.oi.getPilotButton(Controller.B)){
            limelight.trackCargo();
            trackingBay = false;
        }
        if(Robot.oi.getPilotButton(Controller.Y)){
            limelight.trackHatch();
            trackingBay = false;
        }

        double distance = limelight.getDistance(limelight.getPipeline().getWidth());
        double relangle = limelight.getHorizontalAngle();
        table.getEntry("distance").setDouble(distance);
        table.getEntry("relangle").setDouble(relangle);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
