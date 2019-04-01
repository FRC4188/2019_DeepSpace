package robot.commands.groups;

import robot.commands.arm.*;
import robot.commands.intake.*;
import robot.commands.elevator.*;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ToHeight extends CommandGroup {

    boolean lastOnFront = true;

    public enum Height {

        HOME(0.0, 0.0, 0.0),
        CLIMB(2.46, 120.0, 170.0),
        CARGO_LOW(2.25, -112.0, 108.0),
        CARGO_MID(2.25, -55.0, 169.0),
        CARGO_HIGH(2.46, 7.6, 132.0),
        CARGO_SHIP(2.25, -75.0, 185.0),
        CARGO_LOAD(2.25, -112.0, 50.5),
        CARGO_FLOOR(1.15, -105.0, 205.0),
        HATCH_LOW(2.25, -112.0, -39.0),
        HATCH_MID(2.25, -58.0, 0.0),
        HATCH_HIGH(2.46, 27.0, -88.0),
        HATCH_FLOOR(0.52, -114.0, 105.0);

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

        addParallel(new ElevatorToHeight(elevatorHeight, 0.1));
        addParallel(new WristToAngle(wristAngle, 2.0));
        addSequential(new ShoulderToAngle(shoulderAngle, 2.0));

    }

}
