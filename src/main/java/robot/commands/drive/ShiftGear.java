package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.command.Command;

/** Sets gear shift to given value. */
public class ShiftGear extends Command {

    public enum Gear { HIGH, LOW, OFF }

    Drivetrain drivetrain = Robot.drivetrain;
    Gear gear;

    public ShiftGear(Gear gear) {
        requires(Robot.drivetrain);
        setName("ShiftGear: " + gear.toString());
        this.gear = gear;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        if(gear == Gear.HIGH) {
            drivetrain.setHighGear();
            System.out.println("high gear");
        } else if(gear == Gear.LOW) {
            drivetrain.setLowGear();
            System.out.println("low gear");
        } else if(gear == Gear.OFF) {
            drivetrain.setGearShiftOff();
            System.out.println("off");
        }
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }

}
