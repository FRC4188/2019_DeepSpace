package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import robot.utils.CSPMath;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/** Drives forward using pilot left joystick while
 *  turning to keep bay centered. */
public class CenterBay extends Command {

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    final double kP = 0.02;
    final double kD = 0.01;
    final double DELTA_T = 0.02;

    double lastError;

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
        double angleErr = limelight.getHorizontalAngle();
        double turnDeriv = (angleErr - lastError) * DELTA_T;
        double zTurn = kP * angleErr + kD * turnDeriv;
        zTurn = CSPMath.constrainKeepSign(zTurn, 0.05, 1.0);
        lastError = angleErr;

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
