package robot.commands.drive;

import robot.OI;
import robot.Robot;
import robot.OI.Controller;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/** Manually controls drivetrain using pilot controller. */
public class ManualDrive extends Command {

    OI oi = Robot.oi;
    Drivetrain drivetrain = Robot.drivetrain;

    final double kSLOW_TURN = 0.25;
    final double kFAST_TURN = 0.5;
    public double brownoutVariable;

    public ManualDrive() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {

        // get triggers and axes
        double pilotLeftY = oi.getPilotY(Hand.kLeft);
        double pilotRightX = oi.getPilotX(Hand.kRight);
        boolean pilotLeftBumper = oi.getPilotButton(Controller.LB);
        boolean pilotRightBumper = oi.getPilotButton(Controller.RB);

        // turn multiplied by speed unless triggers held
        // left trigger slow turn, right trigger fast
        double xSpeed = pilotLeftY;
        double zTurn = pilotRightX * xSpeed;
        if(pilotLeftBumper) zTurn = pilotRightX * kSLOW_TURN;
        else if(pilotRightBumper) zTurn = pilotRightX * kFAST_TURN;

        drivetrain.arcade(xSpeed * brownoutVariable, zTurn * brownoutVariable);

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
