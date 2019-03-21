package robot.utils;

import robot.commands.drive.*;
import robot.commands.drive.FollowPath.Path;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoChooser {

    private SendableChooser<Integer> chooser;

    public AutoChooser() {

        chooser = new SendableChooser<Integer>();

        chooser.addOption("L1 Far Rocket", 1);
        chooser.addOption("L1 Near Rocket", 2);
        chooser.addOption("L1 Near Ship", 3);
        chooser.addOption("L2 Far Rocket", 4);
        chooser.addOption("L2 Near Rocket", 5);
        chooser.addOption("L2 Near Ship", 6);
        chooser.addOption("M Left Far Rocket", 7);
        chooser.addOption("M Left Near Ship", 8);
        chooser.addOption("M Right Far Rocket", 9);
        chooser.addOption("M Right Near Ship", 10);
        chooser.addOption("R1 Far Rocket", 11);
        chooser.addOption("R1 Near Rocket", 12);
        chooser.addOption("R1 Near Ship", 13);
        chooser.addOption("R2 Far Rocket", 14);
        chooser.addOption("R2 Near Rocket", 15);
        chooser.addOption("R2 Near Ship", 16);

    }

    public Command getSelectedCommand() {

        int selectedOption = chooser.getSelected();

        switch(selectedOption) {
            case 1:
                return new FollowPath(Path.L_TO_L_FAR_ROCKET_HAB1, false);
            case 2:
                return new FollowPath(Path.L_TO_L_NEAR_ROCKET_HAB1, false);
            case 3:
                return new FollowPath(Path.L_TO_L_NEAR_SHIP_HAB1, false);
            case 4:
                return new FollowPath(Path.L_TO_L_FAR_ROCKET_HAB2, false);
            case 5:
                return new FollowPath(Path.L_TO_L_NEAR_ROCKET_HAB2, false);
            case 6:
                return new FollowPath(Path.L_TO_L_NEAR_SHIP_HAB2, false);
            case 7:
                return new FollowPath(Path.M_TO_L_FAR_ROCKET, false);
            case 8:
                return new FollowPath(Path.M_TO_L_NEAR_SHIP, false);
            case 9:
                return new FollowPath(Path.M_TO_R_FAR_ROCKET, false);
            case 10:
                return new FollowPath(Path.M_TO_R_NEAR_SHIP, false);
            case 11:
                return new FollowPath(Path.R_TO_R_FAR_ROCKET_HAB1, false);
            case 12:
                return new FollowPath(Path.R_TO_R_NEAR_ROCKET_HAB1, false);
            case 13:
                return new FollowPath(Path.R_TO_R_NEAR_SHIP_HAB1, false);
            case 14:
                return new FollowPath(Path.R_TO_R_FAR_ROCKET_HAB2, false);
            case 15:
                return new FollowPath(Path.R_TO_R_NEAR_ROCKET_HAB2, false);
            case 16:
                return new FollowPath(Path.R_TO_R_NEAR_SHIP_HAB2, false);
            default:
                return null;
        }

    }

}

