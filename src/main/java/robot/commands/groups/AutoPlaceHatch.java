package robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.commands.intake.FireHatch;
import robot.Robot;
import robot.subsystems.LimeLight;
import robot.utils.*;
import robot.commands.drive.DriveToDistance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class AutoPlaceHatch extends CommandGroup {

    LimeLight limelight = Robot.limelight;
    double dist;
    double tolerance = 0.2;

    public AutoPlaceHatch(){
        dist = limelight.getDistance(limelight.getPipeline().getHeight());
        addSequential(new DriveToDistance(dist, tolerance));
        addParallel(new FireHatch(Value.kReverse));
        addSequential(new Wait(), 0.1);
        addParallel(new FireHatch(Value.kOff));
        addSequential(new Wait(), 0.1);
        addSequential(new DriveToDistance(-dist, 0.5));
    }
}