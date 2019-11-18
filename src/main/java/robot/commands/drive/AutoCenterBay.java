package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import robot.utils.CSPMath;
import edu.wpi.first.wpilibj.command.Command;

public class AutoCenterBay extends Command {

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    final double kP = 0.02;
    final double kD = 0.01;
    final double DELTA_T = 0.02;
    final double acceptableTurn = 0.01;

    double lastError, xSpeed;
    boolean isFinished = false;

    public AutoCenterBay(double xSpeed) {
        requires(drivetrain);
        requires(limelight);
        this.xSpeed = xSpeed;
    }

    @Override
    protected void initialize() {
        limelight.trackBay();
    }

    @Override
    protected void execute() {

        if (Math.abs(zTurn) > acceptableTurn){
            isFinished = false;
        }
        else isFinished = true;

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
        return isFinished;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }

}
