package robot.commands.elevator;

import robot.OI;
import robot.Robot;
import robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/** Manually controls elevator using copilot triggers. */
public class ManualElevator extends Command {

    OI oi = Robot.oi;
    Elevator elevator = Robot.elevator;

    public ManualElevator() {
        requires(elevator);
    }

    @Override
    protected void initialize() {
        double output = oi.getCopilotTrigger(Hand.kLeft);
        if (output > 0){
            elevator.set(output);
        } else {
            output = oi.getCopilotTrigger(Hand.kRight);
            if (output < 0){
                elevator.set(output);
            } else{
                elevator.set(0);
            }
        }
    }

    @Override
    protected void execute() {
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
