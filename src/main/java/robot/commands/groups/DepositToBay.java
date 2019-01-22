package robot.commands.groups;

import robot.commands.drive.*;
import robot.commands.drive.FollowObject.Object;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class DepositToBay extends CommandGroup {
    
    public DepositToBay() {
        addSequential(new FollowObject(Object.BAY_CLOSE));
        addSequential(new FollowLine());
    }

}
