package robot.commands.groups;

import robot.commands.climb.*;
import robot.commands.elevator.*;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbSequence extends CommandGroup {

    public ClimbSequence() {

        addParallel(new ElevatorToVelocity(-0.6));
        addParallel(new ManualClimb(1.0));

    }

}