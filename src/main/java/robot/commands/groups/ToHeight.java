package robot.commands.groups;

import robot.commands.arm.*;
import robot.commands.intake.*;
import robot.commands.elevator.*;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ToHeight extends CommandGroup {

    public enum Height {

        CARGO_LOW(0.0, 0.0, 0.0),
        CARGO_MID(0.0, 0.0, 0.0),
        CARGO_HIGH(0.0, 0.0, 0.0),
        CARGO_SHIP(0.0, 0.0, 0.0),
        CARGO_LOAD(0.0, 0.0, 0.0),
        HATCH_LOW(0.0, 0.0, 0.0),
        HATCH_MID(0.0, 0.0, 0.0),
        HATCH_HIGH(0.0, 0.0, 0.0),
        HATCH_FLOOR(0.0, 0.0, 0.0);

        double elevatorHeight, shoulderAngle, wristAngle;
        Height(double elevatorHeight, double shoulderAngle, double wristAngle) {
            this.elevatorHeight = elevatorHeight;
            this.shoulderAngle = shoulderAngle;
            this.wristAngle = wristAngle;
        }

        double getElevatorHeight() {
            return elevatorHeight;
        }

        double getShoulderAngle() {
            return shoulderAngle;
        }

        double getWristAngle() {
            return wristAngle;
        }

    }

    public ToHeight(Height height) {

        setName("ToHeight: " + height.toString());

        double elevatorHeight = height.getElevatorHeight();
        double shoulderAngle = height.getShoulderAngle();
        double wristAngle = height.getWristAngle();

        addParallel(new ElevatorToHeight(elevatorHeight, 3/12));
        addParallel(new ShoulderToAngle(shoulderAngle, 3));
        addParallel(new WristToAngle(wristAngle, 3));

    }

}
