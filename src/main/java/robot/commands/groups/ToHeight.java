package robot.commands.groups;

import robot.commands.arm.*;
import robot.commands.intake.*;
import robot.commands.elevator.*;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ToHeight extends CommandGroup {

    public enum Height {

        HOME(0.2, -5.0, 0.0),
        CARGO_LOW(2.25, -112.0, 128.0),
        CARGO_MID(2.25, -55.0, 177.0),
        CARGO_HIGH(2.46, 7.6, 152.0),
        CARGO_SHIP(2.25, -75.0, 205.0),
        CARGO_LOAD(2.25, -112.0, 70.5),
        CARGO_FLOOR(1.0, -105.0, 230.0),
        HATCH_LOW(2.25, -112.0, -19.0),
        HATCH_MID(2.25, -58.0, 20.0),
        HATCH_HIGH(2.46, 27.0, -68.0),
        HATCH_FLOOR(0.52, -114.0, 125.0);

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

        addParallel(new ElevatorToHeight(elevatorHeight, 0.05));
        addParallel(new WristToAngle(wristAngle, 1.0));
        addSequential(new ShoulderToAngle(shoulderAngle, 1.0));

    }

}
