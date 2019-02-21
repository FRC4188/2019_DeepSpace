package robot.commands.intake;

import robot.OI;
import robot.Robot;
import robot.subsystems.Intake;
import robot.utils.Brownout;
import robot.utils.CSPMath;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/** Moves intake wrist using y val of right copilot stick. */
public class ManualWrist extends Command {

    OI oi = Robot.oi;
    Intake intake = Robot.intake;
    public double brownoutVariable;

    double lastDir;

    public ManualWrist() {
        requires(intake);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        double percent = oi.getCopilotY(Hand.kRight);
        double wristPos = intake.getWristPosition();
        if(!CSPMath.isBetween(wristPos, -360, 360)) {
            if(Math.signum(percent) == lastDir) percent = 0;
            else percent = oi.getCopilotY(Hand.kRight);
        } else {
            if(Math.abs(percent) > 0) lastDir = Math.signum(percent);
        }
        intake.setWristOpenLoop(percent * brownoutVariable);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }

}
