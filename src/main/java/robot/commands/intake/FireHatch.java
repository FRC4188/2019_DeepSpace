package robot.commands.intake;

import robot.Robot;
import robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

/** Fires hatch solenoid in given direction. kForward ejects hatch. */
public class FireHatch extends Command {

    Intake intake = Robot.intake;
    Value value;

    public FireHatch(Value value) {
        this.value = value;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        intake.setHatchSolenoid(value);
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
