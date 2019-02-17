package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

/** Sets gear shift to given value. kForward is high gear. */
public class ShiftGear extends Command {

    Drivetrain drivetrain = Robot.drivetrain;
    Value value;

    public ShiftGear(Value value) {
        requires(Robot.drivetrain);
        this.value = value;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        drivetrain.shiftGear(value);
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
