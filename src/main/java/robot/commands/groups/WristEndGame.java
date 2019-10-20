package robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.commands.intake.*;

public class WristEndGame extends CommandGroup {

    public WristEndGame() {
        addParallel(new SpinIntake(-1.0));
        addParallel(new WristDown(0.4));
    }

}
