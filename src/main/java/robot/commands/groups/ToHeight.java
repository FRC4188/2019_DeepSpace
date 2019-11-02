package robot.commands.groups;

import robot.commands.arm.*;
import robot.commands.intake.*;
import robot.commands.vision.*;
import robot.commands.elevator.*;
import edu.wpi.first.wpilibj.command.CommandGroup;


public class ToHeight extends CommandGroup {

    public enum Height {

        HOME(0.125, 0.0, 0.0),
        CLIMB(2.46, 120.0, 170.0),
        CARGO_LOW(2.25, -115.0, -261.2),
        CARGO_MID(2.25, -55.1, -202.3),
        CARGO_HIGH(2.48, -8.46, -160),
        CARGO_SHIP(2.25, -50.2, -150.2),
        CARGO_LOAD(2.25, -112.0, 50.5),
        CARGO_FLOOR(1.13, -110.6, -174.4),
        HATCH_LOW(2.34, -115.9, -37.0),
        HATCH_MID(2.25, -58.0, 0.0),
        HATCH_HIGH(2.46, 27.0, -88.0),
        HATCH_FLOOR(0.52, -114.0, 105.0),
        PASS_PREP(2.46, -114.0, 0),
        THROUGH(2.46, -250, 0),
        ENDGAME(2.46, -250, 177);

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

        if(height.equals(Height.HATCH_HIGH)) addParallel(new LimelightBackward());
        else addParallel(new LimelightForward());
        addParallel(new ElevatorToHeight(elevatorHeight, 0.1));
        addParallel(new WristToAngle(wristAngle, 1.2));
        addSequential(new ShoulderToAngle(shoulderAngle, 1.5));

    }

}
