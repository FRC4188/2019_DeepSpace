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
    final double acceptableAng = 0.01;
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
        setGamePiece();
        if (gamePiece != 0){
            while (Math.abs(zTurn) > acceptableAng){
                autoCenterBay();
            }
            setDistance();
            addSequential(new DriveToDistance(dist - 0.5, 0.1));
            if (gamePiece == -1){
                intakeOut();
            }
            else if (gamePiece == 1){
                setHatchState();
                hatchPlace();
            }
            addSequential(new DriveToDistance(-1, 0.5));
        }
    }

    //Sets gamePiece to match Arm's gamePiece
    public void setGamePiece(){
        gamePiece = arm.getGamePiece();
    }

    //Sets hatchState to match Intake's hatchState
    public void setHatchState(){
        hatchState = intake.getHatchState();
    }

    //Sets dist to LimeLight's distance to target
    public void setDistance(){
        dist = limelight.getDistance();
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
        addSequential(new Wait(), 0.8);
        addParallel(new SpinIntake(0));
        addSequential(new Wait(), 0.1);
    }

    //Sequence for deploying/intaking hatches (-1 = out, 1 = in)
    public void hatchPlace(){
        if (hatchState == -1){
            addSequential(new FireHatch(Value.kReverse));
        }
        if (hatchState == 1){
            addSequential(new FireHatch(Value.kForward));
        }
        addParallel(new FireHatch(Value.kOff));
    }
}