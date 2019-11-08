package robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.commands.intake.SpinIntake;
import robot.Robot;
import robot.subsystems.LimeLight;
import robot.utils.*;
import robot.commands.drive.DriveToDistance;

public class AutoPlaceCargo extends CommandGroup {

    LimeLight limelight = Robot.limelight;
    double dist;
    double tolerance = 0.2;

    public AutoPlaceCargo(){
        dist = limelight.getDistance(limelight.getPipeline().getHeight());
        addSequential(new DriveToDistance(dist, tolerance));
        addParallel(new SpinIntake(-1.0));
        addSequential(new Wait(), 0.1);
        addParallel(new SpinIntake(0));
        addSequential(new Wait(), 0.1);
        addSequential(new DriveToDistance(-dist, 0.5));
    }
}