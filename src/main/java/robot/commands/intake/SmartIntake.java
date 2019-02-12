package robot.commands.intake;

import robot.Robot;
import robot.subsystems.Intake;
import robot.subsystems.Intake.IntakeState;
import robot.subsystems.Intake.WristState;
import edu.wpi.first.wpilibj.command.Command;

/** Either sucks in or spits out object based on current orientation of wrist
 *  and whether or not the intake currently has an object. */
public class SmartIntake extends Command {

    Intake intake = Robot.intake;

    WristState wristState;
    IntakeState intakeState;
    final double CARGO_IN_SPEED = -0.5;
    final double CARGO_OUT_SPEED = 0.8;
    boolean isFinished;

    public SmartIntake() {
        requires(intake);
    }

    @Override
    protected void initialize() {
        wristState = intake.getWristState();
        intakeState = intake.getIntakeState();
        isFinished = false;
    }

    @Override
    protected void execute() {

        if(wristState == WristState.CARGO) {

            // wrist in cargo orientation and empty
            if(intakeState == IntakeState.EMPTY) {
                // ball beginning to enter, slow spinning
                if(intake.getFrontCargoSensor()) {
                    intake.spinIntake(CARGO_IN_SPEED * 0.6);
                }
                // ball all the way in, stop spinning
                else if(intake.getFrontCargoSensor() && intake.getRearCargoSensor()) {
                    intake.spinIntake(0);
                    isFinished = true;
                    intake.setIntakeState(IntakeState.FULL);
                }
                // no ball, full speed
                else {
                    intake.spinIntake(CARGO_IN_SPEED);
                }
            }

            // wrist in cargo orientation and full
            if(intakeState == IntakeState.FULL) {
                intake.spinIntake(CARGO_OUT_SPEED);
                // ball gone, stop spinning
                if(!intake.getFrontCargoSensor() && !intake.getRearCargoSensor()) {
                    intake.spinIntake(0);
                    isFinished = true;
                    intake.setIntakeState(IntakeState.EMPTY);
                }
            }

        } else if(wristState == WristState.HATCH) {

            // wrist in hatch orientation and empty
            if(intakeState == IntakeState.EMPTY) {
                intake.hatchCylindersIn();
                isFinished = true;
                intake.setIntakeState(IntakeState.FULL);
            }

            // wrist in hatch orientation and full
            if(intakeState == IntakeState.FULL) {
                intake.hatchCylindersOut();
                isFinished = true;
                intake.setIntakeState(IntakeState.EMPTY);
            }

        }

    }

    @Override
    protected boolean isFinished() {
        return isFinished;
    }

    @Override
    protected void end() {
        intake.spinIntake(0);
        intake.hatchCylindersIn();
        intake.hatchSolenoidOff();
    }

    @Override
    protected void interrupted() {
        end();
    }

}
