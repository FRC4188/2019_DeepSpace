package robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.commands.intake.SpinIntake;
import robot.Robot;
import robot.subsystems.LimeLight;
import robot.utils.*;
import robot.subsystems.Intake;
import robot.commands.drive.DriveToDistance;
import robot.subsystems.Drivetrain;
import robot.subsystems.Arm;
import robot.commands.intake.FireHatch;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class AutoPlace extends CommandGroup {

    LimeLight limelight = Robot.limelight;
    Drivetrain drivetrain = Robot.drivetrain;
    Arm arm = Robot.arm;
    Intake intake = Robot.intake;

    final double kP = 0.02;
    final double kD = 0.01;
    final double DELTA_T = 0.02;
    final double acceptableAng = 0.5;
    double dist;
    int gamePiece, hatchState;
    double lastError = 0;
    double angleErr = limelight.getHorizontalAngle();
    double turnDeriv = (angleErr - lastError) * DELTA_T;
    double zTurn = kP * angleErr + kD * turnDeriv;

    /**
     * If there is no game piece, does nothing.
     * Moves the robot forward autonomously while guiding it with the angle PD loop from CenterBay.
     * Drives forward to close the distance between robot and target.
     * Checks which game piece it has before deploying/intaking hatches or deploying cargo.
     * Moves back from the target afterward.
     */
    public AutoPlace(){
        gamePiece = arm.getGamePiece();
        if (gamePiece != 0){
            while (Math.abs(zTurn) > acceptableAng){
                autoCenterBay();
            }
            dist = limelight.getDistance(limelight.getPipeline().getHeight());
            addSequential(new DriveToDistance(dist, 0.2));
            if (gamePiece == 2){
                intakeOut();
            }
            else if (gamePiece == 1){
                hatchState = intake.getHatchState();
                hatchPlace(hatchState);
            }
            addSequential(new DriveToDistance(-dist, 0.5));
        }
    }

    //CenterBay with autonomous forward motion of constant speed
    public void autoCenterBay(){
        angleErr = limelight.getHorizontalAngle();
        turnDeriv = (angleErr - lastError) * DELTA_T;
        zTurn = kP * angleErr + kD * turnDeriv;
        zTurn = CSPMath.constrainKeepSign(zTurn, 0.05, 1.0);
        lastError = angleErr;
        drivetrain.arcade(0.2, zTurn);
    }

    //Sequence for deploying cargo
    public void intakeOut(){
        addParallel(new SpinIntake(-1.0));
        addSequential(new Wait(), 0.5);
        addParallel(new SpinIntake(0));
        addSequential(new Wait(), 0.1);
    }

    //Sequence for deploying/intaking hatches
    public void hatchPlace(int hatchState){
        if (hatchState == 1){
            addParallel(new FireHatch(Value.kReverse));
        }
        if (hatchState == 2){
            addParallel(new FireHatch(Value.kForward));
        }
        addSequential(new Wait(), 0.1);
        addParallel(new FireHatch(Value.kOff));
        addSequential(new Wait(), 0.1);
    }
}