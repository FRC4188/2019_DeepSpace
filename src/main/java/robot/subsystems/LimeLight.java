package robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import robot.Robot;
import robot.commands.vision.LimeLightUseAsCamera;

/** Limelight vision camera. Used to detect reflective tape. */
public class LimeLight extends Subsystem {

    // limelight network table
    NetworkTable limelightTable = null;

    // distance for target
    private final double TAPE_HEIGHT = 6.0/12;

    // current pipeline
    private Pipeline currentPipeline = Pipeline.OFF;

    // LED mode enum
    public enum LedMode {
        DEFAULT(0), OFF(1), BLINK(2), ON(3);

        private final int value;
        LedMode(int value) {
            this.value = value;
        }
        public int getValue() {
            return this.value;
        }
    }

    // camera mode enum
    public enum CameraMode {
        VISION(0), CAMERA(1);

        private final int value;
        CameraMode(int value) {
            this.value = value;
        }
        public int getValue() {
            return this.value;
        }
    }

    // pipeline enum 
    public enum Pipeline {
        OFF(0, 0.0), CARGO(1, 13.0/12), HATCH(2, 19.0/12),
                BAY_CLOSE(3, 6.0/12), BAY_HIGH(4, 6.0/12);

        private final int value;
        private final double height;

        Pipeline(int value, double height) {
            this.value = value;
            this.height = height;
        }

        public int getValue() {
            return this.value;
        }

        public double getHeight() {
            return this.height;
        }
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new LimeLightUseAsCamera());
    }

    /**
     * Constructor for Limelight.
     */
    public LimeLight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Sets the LED mode of the camera.
     * @param mode the LED mode to set the camera to
     */
    public void setLightMode(LedMode mode) {
        limelightTable.getEntry("ledMode").setNumber(mode.getValue());
    }

    /**
     * Sets the camera mode of the camera.
     * @param mode the camera mode to set the camera to
     */
    public void setCameraMode(CameraMode mode) {
        limelightTable.getEntry("camMode").setNumber(mode.getValue());
    }

    /**
     * Sets the pipeline for the camera to use.
     * @param pl the pipeline for the camera to use
     */
    public void setPipeline(Pipeline pl) {
        limelightTable.getEntry("pipeline").setNumber(pl.getValue());
        currentPipeline = pl;
    }

    /**
     * Returns if the camera sees a target.
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getBoolean(false);
    }

    /**
     * Returns the horizontal angle from the center of the camera to the target.
     */
    public double getHorizontalAngle() {
        return limelightTable.getEntry("tx").getDouble(Robot.drivetrain.getGyroAngle());
    }

    /**
     * Returns the vertical angle from the center of the camera to the target.
     */
    public double getVerticalAngle() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    /** Returns distance in feet from object of height s (feet). 
     *  Uses s = r(theta). */
    public double getDistance(double objectHeight) {
        final double CAMERA_HEIGHT = 240; // pixels
        final double CAMERA_FOV = Math.toRadians(41); // rads
        double boxHeight = limelightTable.getEntry("tvert").getDouble(0.0); // pixels
        if(boxHeight == 0) return 0;
        double percentHeight = boxHeight / CAMERA_HEIGHT;
        double boxDegree = percentHeight * CAMERA_FOV;
        double r = objectHeight / boxDegree; // feet
        return r * 0.95; // fudge boy
    }

    /**
     * Returns distance in feet from object of width given
     */
    public double getDistance(double objectHeight, double boxHeight) {
        if(boxHeight == 0) return 0;
        final double CAMERA_HEIGHT = 240; // pixels
        final double CAMERA_FOV = Math.toRadians(41); // rads
        double percentHeight = boxHeight / CAMERA_HEIGHT;
        double boxDegree = percentHeight * CAMERA_FOV;
        double r = objectHeight / boxDegree; // feet
        return r * 0.95; // fudge lad
    }

    /** Returns necessary distances and turns to get from current location to
     * line perpendicular to vision target, perpLength away. Returns a double array
     * with the angle needed to turn to drive on line to point [0], the distance to 
     * drive to the point [1], and the angle to turn to face perpendicular to target
     * once distance has been driven [2]. Returned in units of feet and degrees. */
    public double[] solvePerpendicular(double perpLength) {

        // get known side lengths and angles (feet and degrees)
        double robotAngle = Robot.drivetrain.getGyroAngle();
        double limelightAngle = getHorizontalAngle();
        double distToTarget = getDistance(getPipeline().getHeight());

        // angle between line from camera to target and perpendicular line in degrees
        // found using parallel lines
        double camToPerpAngle = robotAngle - limelightAngle;

        // solve for distance to point perpendicular to target, perpLength away
        // uses law of cosines
        double distToPerp = Math.sqrt(Math.pow(perpLength, 2) + Math.pow(distToTarget, 2)
                - 2 * perpLength * distToTarget * Math.cos(camToPerpAngle));

        // solve for angle to turn to drive on straight line to point perpendicular to target
        // uses law of sines, returns in degrees
        double botToDistAngle = Math.asin((perpLength * Math.sin(camToPerpAngle)) / distToPerp);

        // solve for angle between distToPerp line and perpendicular line using the formed triangle
        double distToPerpAngle = 180 - botToDistAngle - camToPerpAngle;

        // return values as array
        return (new double[]{
            botToDistAngle, // 0
            distToPerp,     // 1
            distToPerpAngle // 2
        });

    }
    
    /**
     * Returns the pipeline the camera is running
     */
    public Pipeline getPipeline() {
        return currentPipeline;
    }

    /**
     * Start tracking the ship bays
     */
    public void trackShipBay() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        setPipeline(Pipeline.BAY_CLOSE);
    }

    /**
     * Start tracking the closest rocket bays (slightly higher up)
     */
    public void trackRocketBayClose() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        setPipeline(Pipeline.BAY_CLOSE);
    }

    /**
     * Start tracking the hightest rocket bays (slightly higher up)
     */
    public void trackRocketBayHigh() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        setPipeline(Pipeline.BAY_HIGH);
    }

    /**
     * Start tracking the cargo
     */
    public void trackCargo() {
        setLightMode(LedMode.OFF);
        setCameraMode(CameraMode.VISION);
        setPipeline(Pipeline.CARGO);
    }

    /**
     * Start tracking the hatches
     */
    public void trackHatch() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        setPipeline(Pipeline.HATCH);
    }

    /**
     * Use LimeLight as camera
     */
    public void useAsCamera() {
        setLightMode(LedMode.OFF);
        setCameraMode(CameraMode.CAMERA);
        setPipeline(Pipeline.BAY_CLOSE);
    }
}
