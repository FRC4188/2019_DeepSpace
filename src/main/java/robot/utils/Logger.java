package robot.utils;

import badlog.lib.BadLog;
import java.util.Date;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class Logger {

    // Initiates a new BadLog object
    private static BadLog log;
    
    /** Initiates a new badlog file and creates a new .bag file using the
     *  given directory */
    public void init() {
        log = BadLog.init(getLogDir());
        createTopics();
        finishInit();
    }

    /** Pushes updates to the log file - Should be updated periodically */
    public void update() {
        log.updateTopics();
        log.log();
    }

    /** Closes the log file in order to be able to write to it */
    private void finishInit() {
        log.finishInitialization();
    }

    /** Returns the current Timestamp; time is recieved from the
     *  Driver Station */
    private String getTimeStamp() {
        long timeMs = System.currentTimeMillis();
        Date now = new Date(timeMs);
        DateFormat df = new SimpleDateFormat("dd-MM-yyyy HH:mm:ss"); 
        return df.format(now); 
    }

    /** Assigns log directory to /home/lvuser/logs/ and creates a new file
     *  using the timestamp when initiated */
    private String getLogDir() {
        return ("/home/lvuser/logs/" + getTimeStamp() + "_log.bag");
    }

    private void createTopics() {
        BadLog.createTopic("Match Time", "s", () -> DriverStation.getInstance().getMatchTime());
        BadLog.createTopic("Voltage", "V", () -> RobotController.getBatteryVoltage());
    }

}
