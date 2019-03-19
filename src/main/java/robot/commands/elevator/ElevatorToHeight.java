package robot.commands.elevator;

import robot.Robot;
import robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.command.Command;

/** Sets shoulder on arm to given angle in degrees. */
public class ElevatorToHeight extends Command {

    Elevator elevator = Robot.elevator;
    double height, tolerance, counter;

    public ElevatorToHeight(double height, double tolerance) {
        requires(elevator);
        setName("ElevatorToHeight: " + height);
        this.height = height;
        this.tolerance = tolerance;
    }

    @Override
    protected void initialize() {
        counter = 0;
    }

    @Override
    protected void execute() {
        elevator.elevatorToHeight(height, tolerance);
        double error = height - elevator.getPosition();
        if(Math.abs(error) < tolerance) counter++;
        else counter = 0;
    }

    @Override
    protected boolean isFinished() {
        return counter > 10;
    }

    @Override
    protected void end() {
        elevator.set(0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
