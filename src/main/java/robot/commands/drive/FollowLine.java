package robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import edu.wpi.first.wpilibj.AnalogTrigger;

public class FollowLine extends Command {

    private AnalogTrigger lineSensorLeft, lineSensorMid, lineSensorRight;
    private boolean leftSense, midSense, rightSense, offCourse, isDoneFollowing, isFollowing;
    private int state = 0;

    public FollowLine() {
        requires(Robot.m_drivetrain);
    }

    @Override
    protected void initialize() {
        lineSensorLeft = new AnalogTrigger(1);
        lineSensorMid = new AnalogTrigger(2);
        lineSensorRight = new AnalogTrigger(3);

        leftSense = lineSensorLeft.getTriggerState();
        midSense = lineSensorMid.getTriggerState();
        rightSense = lineSensorRight.getTriggerState();
    }

    @Override
    protected void execute() {
        if (leftSense) {
            state += 1;
        }

        if (midSense) {
            state += 2;
        }

        if (rightSense) {
            state += 4;
        }

        if (state > 0) {
            offCourse = true;
        } else {
            offCourse = false;
        }

        switch (state) {
        case 0: // No sensor is not detecting anything
            if (offCourse) {
                System.out.println("The sensor is not detecting anything.");
                isFollowing = false;
            } else {
                System.out.println("Attempting to locate target line");
                Robot.m_drivetrain.arcade(.3, 0, 1.0);
                /** TODO */
            }
            break;
        case 1: // Left
            if (offCourse) {
                System.out.println("Sensing: LEFT; Turning Left");
                Robot.m_drivetrain.arcade(0, -0.3, 1.0);
                /** TODO */
            } else {
                System.out.println("Sensing: LEFT");
                Robot.m_drivetrain.arcade(0, 0.3, 1.0);
                /** TODOs */
            }
            break;
        case 2: // Middle
            System.out.println("Sensing: MID; Continuing Course");
            Robot.m_drivetrain.arcade(0.3, 0, 1.0);
            break;
        case 3: // Middle + Left
            System.out.println("Sensing: MID + LEFT; Adjusting slightly to the left");
            Robot.m_drivetrain.arcade(0, -0.1, 1.0);
            break;
        case 4: // Right
            if (offCourse) {
                System.out.println("Sensing: RIGHT; Turning Right");
                Robot.m_drivetrain.arcade(0, 0.3, 1.0);
                /** TODO */
            } else {
                System.out.println("Sensing: RIGHT");
                Robot.m_drivetrain.arcade(0, -0.3, 1.0);
                /** TODO */
            }
            break;
        case 5: // Right + Left
            System.out.println("Sensing: RIGHT + LEFT; Nothing to do.");
            break;
        case 6: // Right + Mid
            System.out.println("Sensing: RIGHT + MID; Adjusting slightly to the right");
            if (offCourse) {
                Robot.m_drivetrain.arcade(0, 0.1, 1.0);
                /** TODO */
            } else {
                /** TODO */
            }
            break;
        case 7: // Left + Mid + Right
            System.out.println("Sensing: LEFT + MID + RIGHT; Nothing to do.");
            break;
        }

        if (isFollowing) {
            Robot.m_drivetrain.arcade(0.3, 0, 1.0);
            /** TODO: MANAGE MOTOR */
            isDoneFollowing = false;
        } else {
            System.out.println("Finished Following");
            isDoneFollowing = true;
        }
    }

    @Override
    protected boolean isFinished() {
        if (isDoneFollowing == true) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }

}
