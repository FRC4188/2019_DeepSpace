/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.LimeLight.Pipeline;

public class Track extends Command {
  Pipeline pl;
  public Track(Pipeline pipeline) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.limelight);
    pl = pipeline;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    switch(pl){
    case BAY_CLOSE:
      Robot.limelight.trackBay();
      break;
    case BAY_3D:
      Robot.limelight.trackBay3D();
      break;
    case CARGO:
      Robot.limelight.trackCargo();
      break;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
