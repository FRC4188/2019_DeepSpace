package robot.commands.drive;

import java.io.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.Drivetrain;

/** Records velocities for each side of the drivetrain
 *  to a CSV file while manually driving. */
public class RecordTrajectory extends Command {

    Drivetrain drivetrain = Robot.drivetrain;
    private final double DELTA_T = 0.02; // seconds
    private Notifier notif = new Notifier(() -> record());
    File file;
    FileWriter fw;
    BufferedWriter bw;

    public RecordTrajectory() {
    }

    @Override
    protected void initialize() {

        // get name of recording
        String fileName = SmartDashboard.getString("Trajectory Recording Name", null);

        // set up file and start recording if file name exists
        if(fileName != null) {
            file = new File("/home/lvuser/" + fileName + ".csv");
            try {
                fw = new FileWriter(file);
                bw = new BufferedWriter(fw);
                bw.write("lVoltage,rVoltage,lVel,rVel");
            } catch(IOException e) {
                e.printStackTrace();
            }
            notif.startPeriodic(DELTA_T);
        } else {
            System.out.println("File name empty!");
        }

    }

    protected void record() {

        // get applied voltages from controllers
        double lVoltage = drivetrain.getLeftAppliedVoltage();
        double rVoltage = drivetrain.getRightAppliedVoltage();

        // get drive velocities from controllers
        double lVel = drivetrain.getLeftVelocity();
        double rVel = drivetrain.getRightVelocity();

        // format output
        String line = lVoltage + "," + rVoltage + "," + lVel + "," + rVel + "\n";

        // write to file
        try {
            bw.write(line);
        } catch(IOException e) {
            e.printStackTrace();
        }

    }

    @Override
    protected boolean isFinished() {
        return SmartDashboard.getBoolean("Stop Recording Trajectory", false);
    }

    @Override
    protected void end() {
        notif.stop();
        try {
            bw.close();
            fw.close();
        } catch(IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void interrupted() {
        end();
    }

}