package robot.commands.arm;

import robot.Robot;
import robot.subsystems.Arm;
import robot.subsystems.Drivetrain;
import robot.subsystems.Elevator;
import robot.subsystems.Intake;
import edu.wpi.first.wpilibj.command.Command;

/** Sets shoulder encoders to 0. */
public class ZeroArm extends Command {

    Arm arm = Robot.arm;
    Intake intake = Robot.intake;
    Elevator elevator = Robot.elevator;
    Drivetrain drivetrain = Robot.drivetrain;

    public ZeroArm() {
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        arm.resetEncoders();
        intake.resetEncoders();
        elevator.resetEncoders();
        drivetrain.resetEncoders();
        drivetrain.resetGyro();
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
