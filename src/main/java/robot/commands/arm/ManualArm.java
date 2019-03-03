package robot.commands.arm;

import robot.OI;
import robot.Robot;
import robot.subsystems.Arm;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Manually controls arm using left copilot Y. */
public class ManualArm extends Command {

    OI oi = Robot.oi;
    Arm arm = Robot.arm;

    public ManualArm() {
        requires(arm);
    }

    @Override
    protected void initialize() {
        SmartDashboard.putBoolean("Arm closed loop", true);
    }

    @Override
    protected void execute() {
        double brownoutVar = Robot.brownoutProtection.getBrownoutVar();
        boolean isClosedLoop = SmartDashboard.getBoolean("Arm closed loop", true);
        if(isClosedLoop) {
            System.out.println("arm is closed loop");
            arm.set(oi.getCopilotY(Hand.kLeft) * brownoutVar);
        } else {
            System.out.println("arm is open loop");
            arm.setOpenLoop(oi.getCopilotY(Hand.kLeft) * brownoutVar, false);
        }
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
