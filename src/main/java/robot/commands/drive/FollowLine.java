package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.command.Command;

/** Follows line of reflective tape using photo sensors. */
public class FollowLine extends Command {

    Drivetrain drivetrain = Robot.drivetrain;

    final double SPEED = 0.25;
    final double TURN_MINOR = 0.1;
    final double TURN_MAJOR = 0.2;
    boolean isFollowingLine;
    double lastLineTurn;

    public FollowLine() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        isFollowingLine = false;
        lastLineTurn = 0;
    }

    @Override
    protected void execute() {

        // get data from photo sensors, true = reflecting
        boolean leftSense = drivetrain.getLeftLineSensor();
        boolean midSense = drivetrain.getMidLineSensor();
        boolean rightSense = drivetrain.getRightLineSensor();

        // drive forward until line is detected, then begin control loop
        // lastLineTurn stores the direction needed to turn while reversing if line is lost
        if(!isFollowingLine) {
            System.out.println("Not tracking line, continuing straight.");
            drivetrain.arcade(SPEED, 0, 1.0);
            if(leftSense || rightSense || midSense) isFollowingLine = true;
        } else {
            if(leftSense && !midSense && !rightSense) {         // left only
                System.out.println("Only left sensing, major turn right.");
                drivetrain.arcade(SPEED, -TURN_MAJOR, 1.0);
                lastLineTurn = -1;
            } else if(leftSense && midSense && !rightSense) {   // left and mid
                System.out.println("Left and mid sensing, minor turn right.");
                drivetrain.arcade(SPEED, -TURN_MINOR, 1.0);
                lastLineTurn = -1;
            } else if(!leftSense && midSense && !rightSense) {  // mid only
                System.out.println("Only mid sensing, continuing straight.");
                drivetrain.arcade(SPEED, 0, 1.0);
                lastLineTurn = 0;
            } else if(!leftSense && midSense && rightSense) {   // right and mid
                System.out.println("Right and mid sensing, minor turn left.");
                drivetrain.arcade(SPEED, TURN_MINOR, 1.0);
                lastLineTurn = 1;
            } else if(!leftSense && !midSense && rightSense) {  // right only
                System.out.println("Only right sensing, major turn left.");
                drivetrain.arcade(SPEED, TURN_MAJOR, 1.0);
                lastLineTurn = 1;
            } else if(leftSense && !midSense && rightSense) {   // left and right
                System.out.println("Left and right sensing, continuing straight.");
                drivetrain.arcade(SPEED, 0, 1.0);
                lastLineTurn = 0;
            } else if(leftSense && midSense && rightSense) {    // all three
                System.out.println("All three sensing, continuing straight.");
                drivetrain.arcade(SPEED, 0, 1.0);
                lastLineTurn = 0;
            } else {
                String lastSense;
                if(lastLineTurn == 0) {
                    drivetrain.arcade(SPEED, 0, 1.0);
                    System.out.println("Lost line, driving forward.");
                } else {
                    lastSense = (lastLineTurn == 1) ? "right" : "left";
                    drivetrain.arcade(-SPEED, lastLineTurn * TURN_MAJOR, 1.0);
                    System.out.println("Lost line, driving backwards and " + lastSense);
                }
            }
        }

    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        drivetrain.tank(0, 0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
