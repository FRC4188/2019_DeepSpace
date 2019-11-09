package robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.commands.intake.FireHatch;
import robot.Robot;
import robot.subsystems.LimeLight;
import robot.utils.*;
import robot.commands.drive.DriveToDistance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import robot.subsystems.Drivetrain;

public class AutoPlaceHatch extends CommandGroup {

    LimeLight limelight = Robot.limelight;
    Drivetrain drivetrain = Robot.drivetrain;

    final double kP = 0.02;
    final double kD = 0.01;
    final double DELTA_T = 0.02;
    double dist;
    double tolerance = 0.2;
    double lastError = 0;
    double angleErr = limelight.getHorizontalAngle();
    double turnDeriv = (angleErr - lastError) * DELTA_T;
    double zTurn = kP * angleErr + kD * turnDeriv;

    public AutoPlaceHatch(){
        while (Math.abs(zTurn) > 0.7){
            autoCenterBay();
        }
        dist = limelight.getDistance(limelight.getPipeline().getHeight());
        addSequential(new DriveToDistance(dist, tolerance));
        addParallel(new FireHatch(Value.kReverse));
        addSequential(new Wait(), 0.1);
        addParallel(new FireHatch(Value.kOff));
        addSequential(new Wait(), 0.1);
        addSequential(new DriveToDistance(-dist, 0.5));
    }

    public void autoCenterBay(){
        angleErr = limelight.getHorizontalAngle();
        turnDeriv = (angleErr - lastError) * DELTA_T;
        zTurn = kP * angleErr + kD * turnDeriv;
        zTurn = CSPMath.constrainKeepSign(zTurn, 0.05, 1.0);
        lastError = angleErr;
        drivetrain.arcade(0.2, zTurn);
    }
}