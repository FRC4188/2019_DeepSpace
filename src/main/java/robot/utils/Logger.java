package robot.utils;

import badlog.lib.BadLog;
import java.util.Date;
import java.text.DateFormat;
import java.text.SimpleDateFormat;

public class Logger {

    private static BadLog log;

    private String getTimeStamp() {
        long timeMs = System.currentTimeMillis();
        Date now = new Date(timeMs);
        DateFormat df = new SimpleDateFormat("dd-MM-yyyy HH:mm:ss"); 
        return df.format(now); 
    }

    private String getLogDir() {
        return ("/home/lvuser/logs/" + getTimeStamp() + "_log.bag");
    }

    public void update() {
        log.updateTopics();
        log.log();
    }

    public void init() {
        log = BadLog.init(getLogDir());
    }

    public void finishInit() {
        log.finishInitialization();
    }

}
