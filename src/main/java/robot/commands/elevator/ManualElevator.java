package robot.commands.elevator;

import robot.OI;
import robot.Robot;
import robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/** Manually controls elevator using copilot triggers (left up, right down). */
public class ManualElevator extends Command {

    OI oi = Robot.oi;
    Elevator elevator = Robot.elevator;
    public double brownoutVariable;

    public ManualElevator() {
        requires(elevator);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        double leftTrigger = oi.getCopilotTrigger(Hand.kLeft);
        double rightTrigger = oi.getCopilotTrigger(Hand.kRight);
        if (leftTrigger > 0) elevator.set(-leftTrigger * brownoutVariable);
        else if(rightTrigger > 0) elevator.set(rightTrigger * brownoutVariable);
        else elevator.set(0);
    }

    @Override
    protected boolean isFinished() {
        return false;
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
