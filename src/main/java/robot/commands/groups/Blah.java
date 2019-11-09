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

public class Blah extends CommandGroup {

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
     * Drives forward for to close the distance between robot and target.
     * Checks which game piece it has before deploying/intaking hatches or deploying cargo.
     * Moves back from the target afterward.
     */
    public Blah(){
        setGamePiece();
        if (gamePiece == 2){
            System.out.println("Cargo");
            intakeOut();
        }
        else if (gamePiece == 1){
            setHatchState();
            if (hatchState == -1){
                System.out.println("Hatch out");
                addParallel(new FireHatch(Value.kReverse));
            }
            else if (hatchState == 1){
                System.out.println("Hatch in");
                addParallel(new FireHatch(Value.kForward));
            }
            addSequential(new FireHatch(Value.kOff));
        }
        //addSequential(new DriveToDistance(-1, 0.5));
    }

    //Sets gamePiece to match Arm's gamePiece
    public void setGamePiece(){
        gamePiece = arm.getGamePiece();
    }

    //Sets hatchState to match Intake's hatchState
    public void setHatchState(){
        hatchState = intake.getHatchState();
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

    //Sequence for deploying/intaking hatches
    public void hatchPlace(){
        if (hatchState == -1){
            System.out.println("Hatch out");
            addParallel(new FireHatch(Value.kReverse));
        }
        else if (hatchState == 1){
            System.out.println("Hatch in");
            addParallel(new FireHatch(Value.kForward));
        }
        addSequential(new FireHatch(Value.kOff));
    }
  }