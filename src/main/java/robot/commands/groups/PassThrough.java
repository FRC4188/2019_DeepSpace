package robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.commands.groups.ToHeight;
import robot.commands.groups.ToHeight.Height;

public class PassThrough extends CommandGroup {

    public PassThrough() {
        addSequential(new ToHeight(Height.PASS_PREP));
        addSequential(new ToHeight(Height.THROUGH));
        addSequential(new ToHeight(Height.ENDGAME));
    }

}