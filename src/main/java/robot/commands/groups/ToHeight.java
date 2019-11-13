package robot.commands.groups;

import robot.commands.arm.*;
import robot.commands.intake.*;
import robot.commands.vision.*;
import robot.subsystems.Arm;
import robot.commands.elevator.*;
import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.Robot;

public class ToHeight extends CommandGroup {

    Arm arm = Robot.arm;

    public enum Height {

        /**
         * Added a new int parameter called game piece, determines whether AutoPlace does 
         * cargo(-1), hatch(1), or nothing(0). 
         */
        HOME(0.125, 0.0, 0.0, 0),
        CLIMB(2.46, 120.0, 170.0, 0),
        CARGO_LOW(2.25, -115.0, -261.2, -1),
        CARGO_MID(2.25, -55.1, -202.3, -1),
        CARGO_HIGH(2.48, -8.46, -160, -1),
        CARGO_SHIP(2.25, -50.2, -150.2, -1),
        CARGO_LOAD(2.25, -112.0, 50.5, 0),
        CARGO_FLOOR(1.13, -110.6, -174.4, 0),
        HATCH_LOW(2.34, -115.9, -37.0, 1),
        HATCH_MID(2.25, -58.0, 0.0, 1),
        HATCH_HIGH(2.46, 27.0, -88.0, 1),
        HATCH_FLOOR(0.52, -114.0, 105.0, 0),
        PASS_PREP(2.46, -114.0, 0, 0),
        THROUGH(2.46, -250, 0, 0),
        ENDGAME(2.46, -250, 177, 0);

        double elevatorHeight, shoulderAngle, wristAngle;
        int gamePiece;
        Height(double elevatorHeight, double shoulderAngle, double wristAngle, int gamePiece) {
            this.elevatorHeight = elevatorHeight;
            this.shoulderAngle = shoulderAngle;
            this.wristAngle = wristAngle;
            this.gamePiece = gamePiece;
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

        int getGamePiece(){
            return gamePiece;
        }
    }

    public ToHeight(Height height) {

        setName("ToHeight: " + height.toString());

        double elevatorHeight = height.getElevatorHeight();
        double shoulderAngle = height.getShoulderAngle();
        double wristAngle = height.getWristAngle();
        int gamePiece = height.getGamePiece();

        if(height.equals(Height.HATCH_HIGH)) addParallel(new LimelightBackward());
        else addParallel(new LimelightForward());
        addParallel(new SetGamePiece(gamePiece));
        addParallel(new ElevatorToHeight(elevatorHeight, 0.1));
        addParallel(new WristToAngle(wristAngle, 1.2));
        addSequential(new ShoulderToAngle(shoulderAngle, 1.5));
    }

}
