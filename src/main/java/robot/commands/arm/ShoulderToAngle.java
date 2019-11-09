package robot.commands.arm;

import robot.Robot;
import robot.subsystems.Arm;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Sets shoulder on arm to given angle in degrees. */
public class ShoulderToAngle extends Command {

    Arm arm = Robot.arm;
    double angle, tolerance, counter, trim;
    int gamePiece;

    public ShoulderToAngle(double angle, double tolerance, int gamePiece) {
        requires(arm);
        setName("ShoulderToAngle: " + angle);
        SmartDashboard.putNumber("Arm trim", trim);
        this.angle = angle;
        this.tolerance = tolerance;
        this.gamePiece = gamePiece;
    }

    @Override
    protected void initialize() {
        counter = 0;
        trim = 0;
    }

    @Override
    protected void execute() {
        trim = SmartDashboard.getNumber("Arm trim", 0);
        arm.shoulderToAngle(angle + trim, tolerance, gamePiece);
        double error = angle - arm.getPosition();
        if(Math.abs(error) < tolerance) counter++;
        else counter = 0;
    }

    @Override
    protected boolean isFinished() {
        return counter > 10;
    }

    @Override
    protected void end() {
        arm.set(0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
