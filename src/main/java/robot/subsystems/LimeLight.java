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
    private final double TAPE_WIDTH = 3.25;

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
                BAY_CLOSE(3, 15.0/12), BAY_HIGH(4, 15.0/12);

        private final int value;
        private final double width;

        Pipeline(int value, double width) {
            this.value = value;
            this.width = width;
        }

        public int getValue() {
            return this.value;
        }

        public double getWidth() {
            return this.width;
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

    /** Returns distance in feet from object of width s (feet). 
     *  Uses s = r(theta). */
    public double getDistance(double objectWidth) {
        final double CAMERA_WIDTH = 320; // pixels
        final double CAMERA_FOV = Math.toRadians(54); // rads
        double boxWidth = limelightTable.getEntry("thor").getDouble(0.0); // pixels
        if(boxWidth == 0) return 0;
        double percentWidth = boxWidth / CAMERA_WIDTH;
        double boxDegree = percentWidth * CAMERA_FOV;
        double r = objectWidth / boxDegree; // feet
        return r;
    }

    /**
     * Returns distance in feet from object of width given
     */
    public double getDistance(double objectWidth, double boxWidth) {
        if(boxWidth == 0) return 0;
        final double CAMERA_WIDTH = 320; // pixels
        final double CAMERA_FOX = Math.toRadians(54); // rads
        double percentWidth = boxWidth / CAMERA_WIDTH;
        double boxDegree = percentWidth * CAMERA_FOX;
        double r = objectWidth / boxDegree;
        return r;
    }

    /**
     * Returns a guesstimate angle to try and square up on bay by comparing size
     * of left and right contours.
     */
    public double getCorrectionAngle() {
        // ensure we are tracking bays
        if(!(currentPipeline == Pipeline.BAY_CLOSE || currentPipeline == Pipeline.BAY_HIGH)) return 0;
        // get angles to raw contours (rough guess)
        double leftArea, rightArea;
        if(limelightTable.getEntry("tx0").getDouble(0.0) < limelightTable.getEntry("tx1").getDouble(0.0)) {
            // 0 is left, 1 is right
            leftArea = limelightTable.getEntry("tvert0").getDouble(0.0);
            rightArea = limelightTable.getEntry("tvert1").getDouble(0.0);
        } else {
            // 1 is left, 0 is right
            leftArea = limelightTable.getEntry("tvert1").getDouble(0.0);
            rightArea = limelightTable.getEntry("tvert0").getDouble(0.0);
        }
        final double threshold = Math.log10(1.1);
        double areaRatio = leftArea / rightArea;
        SmartDashboard.putNumber("ratio", areaRatio);
        double logRatio = Math.log10(areaRatio); // positive : turn right
        if(Math.abs(logRatio) < threshold) return 0; // camera is centered enough to not need correction
        return logRatio * 200; // guess to get angle from ratio
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
        double distToTarget = getDistance(getPipeline().getWidth());

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
