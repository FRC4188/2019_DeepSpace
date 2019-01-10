/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class LimeLightUseAsCamera extends Command {
  public LimeLightUseAsCamera() {
    requires(Robot.m_limelight);
  }

  @Override
  protected void initialize() {
    Robot.m_limelight.useAsCamera();
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
