package robot.commands.rumble;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import robot.OI;;

public class MillisecondRumble extends Command {
//Initialized the OI and long//
    private long _duration;
    private long _endTime;
    private OI _oi;
//Initialized the intensities//
    private double coPilotLeftIntensity;
    private double coPilotRightIntensity;
    private double pilotLeftIntensity;
    private double pilotRightIntensity;
//Created the objects for this command//
    public MillisecondRumble(OI oi, long duration, double intensity) {
        _oi = oi;
        _duration = duration;
        coPilotLeftIntensity = intensity;
        coPilotRightIntensity = intensity;
        pilotLeftIntensity= intensity;
        pilotRightIntensity = intensity;
    }
//Initialize the duration//
    @Override
    protected void initialize() {
        _endTime = System.currentTimeMillis() + _duration;
        _oi.setPilotRumble(pilotLeftIntensity, pilotRightIntensity);
        _oi.setCopilotRumble(coPilotLeftIntensity, coPilotRightIntensity);
        DriverStation.reportError("Rumble initializing. Rumbling for " + Long.toString(_duration) + "ms", false);
    }
//Stops the rumble and turns it to an intensity of 0//
    @Override
    protected void end() {
        _oi.setPilotRumble(0);
        _oi.setCopilotRumble(0);
        DriverStation.reportError("Rumble ended.", false);
    }

    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _endTime;
    }
}