package robot.commands.drive;

import java.io.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.Drivetrain;

/** Plays back a trajectory by commanding motor output
 *  based on values from a recorded CSV file of given file name. */
public class PlayTrajectory extends Command {

    Drivetrain drivetrain = Robot.drivetrain;
    private final double DELTA_T = 0.02; // seconds
    private Notifier notif = new Notifier(() -> play());
    File file;
    FileReader fr;
    BufferedReader br;
    boolean isFinished;

    public PlayTrajectory(String fileName) {
        file = new File("/home/lvuser/" + fileName + ".csv");
    }

    @Override
    protected void initialize() {

        isFinished = false;

        // disable ramp rate
        drivetrain.disableRampRate();

        // set up file and start recording if file name exists
        if(file.isFile()) {
            try {
                fr = new FileReader(file);
                br = new BufferedReader(fr);
                br.readLine();
            } catch(IOException e) {
                e.printStackTrace();
            }
            notif.startPeriodic(DELTA_T);
        } else {
            System.out.println("File doesn't exist!");
        }

    }

    protected void play() {

        String line = null;
        try {
            while((line = br.readLine()) != null) {

                // get data and seperate values
                String[] data = line.split(",");
                double lVoltage = Double.parseDouble(data[0]);
                double rVoltage = Double.parseDouble(data[1]);
                double lVel = Double.parseDouble(data[2]);
                double rVel = Double.parseDouble(data[3]);

                // normalize voltage values to [-1,1] scale
                double l = lVoltage / drivetrain.getLeftInputVoltage();
                double r = rVoltage / drivetrain.getRightInputVoltage();

                // command motor output
                drivetrain.tank(l, r);

            }
            isFinished = true;
        } catch(IOException e) {
            e.printStackTrace();
        }

    }

    @Override
    protected boolean isFinished() {
        return isFinished;
    }

    @Override
    protected void end() {
        drivetrain.tank(0, 0);
        drivetrain.enableRampRate();
        notif.stop();
        try {
            br.close();
            fr.close();
        } catch(IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void interrupted() {
        end();
    }

}