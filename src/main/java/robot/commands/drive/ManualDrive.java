package robot.commands.drive;

import robot.OI;
import robot.Robot;
import robot.OI.Controller;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/** Manually controls drivetrain using pilot controller. */
public class ManualDrive extends Command {

    OI oi = Robot.oi;
    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    final double kSLOW_TURN = 0.25;
    final double kFAST_TURN = 0.5;
    final double kSTICK_TURN = 0.6;
    final double VISION_kP = 0.01;

    public ManualDrive() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {

        // get values
        double pilotLeftY = oi.getPilotY(Hand.kLeft);
        double pilotRightX = oi.getPilotX(Hand.kRight);
        boolean pilotLeftBumper = oi.getPilotButton(Controller.LB);
        boolean pilotRightBumper = oi.getPilotButton(Controller.RB);
        boolean pilotRightTrigger = oi.getPilotTrigger(Hand.kRight) > 0.5;
        double brownoutVar = Robot.brownoutProtection.getBrownoutVar();

        // turn multiplied by speed unless bumpers held
        // left bumper slow turn, right bumper fast
        double xSpeed = pilotLeftY;
        double zTurn = pilotRightX * Math.abs(xSpeed) * kSTICK_TURN;
        if(pilotLeftBumper) zTurn = pilotRightX * kSLOW_TURN;
        else if(pilotRightBumper) zTurn = pilotRightX * kFAST_TURN;

        // if right trigger held, turn to center bay target
        if(pilotRightTrigger) {
            limelight.trackBay();
            double camHorizAngle = limelight.getHorizontalAngle();
            zTurn = VISION_kP * camHorizAngle;
        }

        // command motor output
        drivetrain.arcade(xSpeed * brownoutVar, zTurn * brownoutVar);

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
