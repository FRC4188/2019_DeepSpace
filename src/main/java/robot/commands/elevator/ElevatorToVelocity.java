package robot.commands.elevator;

import robot.Robot;
import robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.command.Command;

/** Sets elevator to a given velocity in feet per second. */
public class ElevatorToVelocity extends Command {

    Elevator elevator = Robot.elevator;
    double velocity;

    public ElevatorToVelocity(double velocity) {
        requires(elevator);
        setName("ElevatorToVelocity: " + velocity);
        this.velocity = velocity;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        elevator.set(velocity);
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
