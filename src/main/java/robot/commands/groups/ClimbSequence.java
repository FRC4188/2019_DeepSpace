package robot.commands.groups;

import robot.commands.arm.*;
import robot.commands.climb.*;
import robot.commands.intake.*;
import robot.commands.elevator.*;
import robot.commands.groups.ToHeight.Height;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbSequence extends CommandGroup {

    public ClimbSequence() {

        addSequential(new ToHeight(Height.CLIMB), 2.0);
        addParallel(new HoldArmAngle(), 5.0);
        addParallel(new ElevatorToHeight(0.65, 0.01), 2.0);
        addParallel(new ManualClimb(0.43), 2.0);
        addSequential(new SpinIntake(0.3), 5.0);
        //addSequential(new ToHeight(Height.HOME), 2.0);

    }

}
