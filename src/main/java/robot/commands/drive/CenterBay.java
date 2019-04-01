package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/** Drives forward using pilot left joystick while
 *  turning to keep bay centered. */
public class CenterBay extends Command {

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    final double TURN_kP = 0.013;

    public CenterBay() {
        requires(drivetrain);
        requires(limelight);
    }

    @Override
    protected void initialize() {
        limelight.trackBay();
    }

    @Override
    protected void execute() {

        // use pilot left joystick for xSpeed
        double xSpeed = Robot.oi.getPilotY(Hand.kLeft);

        // angle p loop
        double angleErr = limelight.getHorizontalAngle() - 2.5;
        double zTurn = TURN_kP * angleErr;

        // command motor output
        drivetrain.arcade(xSpeed, zTurn);

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
