package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import robot.utils.CSPMath;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;

/** Turns to given angle in degrees using PID loop. */
public class TurnToAngle extends Command {

    public enum Angle { RELATIVE, ABSOLUTE, PERP_FIRST, TARGET }

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    final double kP = 0.01;
    final double kI = 0;
    final double kD = 0;

    double lastError, integral = 0;
    double angle, tolerance, angleParam;
    Angle type;

    public TurnToAngle(double angle, double tolerance, Angle type) {
        requires(Robot.drivetrain);
        this.angleParam = angle;
        this.tolerance = tolerance;
        this.type = type;
    }

    /** If type is FIRST, turns to the first angle to get on
     *  line to perpendicular point. If TARGET, turns to target. */
    public TurnToAngle(Angle type) {
        requires(Robot.drivetrain);
        requires(Robot.limelight);
        this.tolerance = 3.0;
        this.type = type;
    }

    @Override
    protected void initialize() {

        // determine setpoint based on type
        if(type == Angle.RELATIVE) angle = angleParam + drivetrain.getGyroAngle();
        else if(type == Angle.PERP_FIRST) angle = limelight.solvePerpendicular()[0];
        else if(type == Angle.TARGET) angle = limelight.solvePerpendicular()[2];
        else angle = angleParam;

        // reset fields and ensure angle is in range
        lastError = 0;
        integral = 0;
        angle = Pathfinder.boundHalfDegrees(angle);

    }

    @Override
    protected void execute() {
        double input = drivetrain.getGyroAngle();
        double error = angle - input;
        if(Math.abs(error) > 180) error = (error > 0) ? error - 360 : error + 360; // take shortest path to angle
        integral += error * drivetrain.DELTA_T;
        double derivative = (error - lastError) / drivetrain.DELTA_T;
        double output = kP * error + kI * integral + kD * derivative;
        output = CSPMath.constrainKeepSign(output, 0.21, 1.0);
        lastError = error;
        drivetrain.tank(output, -output, 1.0);
    }

    @Override
    protected boolean isFinished() {
        return (Math.abs(lastError) < tolerance);
    }

    @Override
    protected void end() {
        drivetrain.tank(0, 0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
