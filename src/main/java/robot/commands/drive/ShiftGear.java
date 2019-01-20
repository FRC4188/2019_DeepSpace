package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

/** Toggles gear shift solenoid. */
public class ShiftGear extends Command {

    Drivetrain drivetrain = Robot.drivetrain;
    boolean lastState;

    public ShiftGear() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        lastState = !lastState;
    }

    @Override
    protected void execute() {
        if(lastState) {
            drivetrain.shiftGear(Value.kForward);
            System.out.println("Gearshift set to forward");
        } else {
            drivetrain.shiftGear(Value.kReverse);
            System.out.println("Gearshift set to reverse");
        }
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
