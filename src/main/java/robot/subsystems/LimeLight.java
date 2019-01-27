package robot.subsystems;

import robot.Robot;
import robot.commands.vision.LimeLightUseAsCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;

/** Limelight vision camera. Used to detect reflective tape. */
public class LimeLight extends Subsystem {

    // limelight network table
    NetworkTable limelightTable = null;

    // distance for target
    private final double TAPE_HEIGHT = 6.0/12;
    private final double HORIZONTAL_SCALE = Math.tan(Math.toRadians(27));
    private final double VERTICAL_SCALE = Math.tan(Math.toRadians(20.5));

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

    /**
     * Returns the robot angle relative to the wall.
     */
    public double getRobotAngle(){
        // ensure we are tracking a bay
        if(!(currentPipeline == Pipeline.BAY_CLOSE || currentPipeline == Pipeline.BAY_HIGH)) return 0;
        //double horAngle0 = Math.toRadians(limelightTable.getEntry("tx0").getDouble(0.0) * 27); // rad
        //double horAngle1 = Math.toRadians(limelightTable.getEntry("tx1").getDouble(0.0) * 27); // rad
        double horAngle0 = Math.toRadians(Math.atan(HORIZONTAL_SCALE * limelightTable.getEntry("tx0").getDouble(0.0)));
        double horAngle1 = Math.toRadians(Math.atan(HORIZONTAL_SCALE * limelightTable.getEntry("tx1").getDouble(0.0)));
        double tvert0 = limelightTable.getEntry("tvert0").getDouble(0.0); // ft
        double tvert1 = limelightTable.getEntry("tvert1").getDouble(0.0); // ft
        // get the distance and the horizontal angle of the two raw contours
        double leftAngle, rightAngle; // rad
        double leftDistance, rightDistance; // ft
        if(horAngle0 < horAngle1){
            // 0 is left, 1 is right
            leftAngle = horAngle0;
            rightAngle = horAngle1;
            leftDistance = getDistance(TAPE_HEIGHT, tvert0);
            rightDistance = getDistance(TAPE_HEIGHT, tvert1);
        } else {
            // 1 is left, 0 is right
            leftAngle = horAngle1;
            rightAngle = horAngle0;
            leftDistance = getDistance(TAPE_HEIGHT, tvert1);
            rightDistance = getDistance(TAPE_HEIGHT, tvert0);
        }
        // calculate the distance between the tape strips, and find the angle
        double horzDiff = rightDistance * Math.sin(rightAngle) - leftDistance * Math.sin(leftAngle);
        double vertDiff = rightDistance * Math.cos(rightAngle) - leftDistance * Math.cos(leftAngle);
        return Math.toDegrees(Math.atan2(vertDiff, horzDiff));
    }

    /** Returns necessary distances and turns to get from current location to
     * line perpendicular to vision target, perpLength away. Returns a double array
     * with the angle needed to turn to drive on line to point [0], the distance to 
     * drive to the point [1], and the angle to turn to face perpendicular to target
     * once distance has been driven [2]. Returned in units of feet and degrees. */
    public double[] solvePerpendicular(double perpLength) {
<<<<<<< HEAD
        // ensure we are tracking a bay
        if(!(currentPipeline == Pipeline.BAY_CLOSE || currentPipeline == Pipeline.BAY_HIGH)) return new double[]{0.0, 0.0, 0.0};
        // get data
        double distance = getDistance(currentPipeline.getHeight());
        double relativeAngle = getHorizontalAngle();
        double robotAngle = getRobotAngle();
        // calculate the distances to the destination
        double horzDiff = distance * Math.sin(Math.toRadians(relativeAngle + robotAngle)); // feet
        double vertDiff = distance * Math.cos(Math.toRadians(relativeAngle + robotAngle)) - perpLength; // feet
        double turnAngle = Math.toDegrees(Math.atan2(horzDiff , vertDiff)) - robotAngle;
        double newDistance = Math.sqrt(horzDiff * horzDiff + vertDiff * vertDiff);
        double secondTurnAngle = -Math.toDegrees(Math.atan2(vertDiff, horzDiff));
        return new double[]{turnAngle, newDistance, secondTurnAngle};
=======

        // estimate field relative target angle based off current heading
        double targetAngle = 90;

        // get known side lengths and angles (feet and degrees)
        // all angles relative to target, not field
        double robotAngle = Robot.drivetrain.getGyroAngle() - targetAngle;
        double limelightAngle = getHorizontalAngle();
        double distToTarget = getDistance(getPipeline().getHeight());

        // angle between line from camera to target and perpendicular line in radians
        // found using parallel lines
        double camToPerpAngle = Math.toRadians(robotAngle - limelightAngle);

        // solve for distance to point perpendicular to target, perpLength away
        // uses law of cosines
        double driveDist = Math.sqrt(Math.pow(perpLength, 2) + Math.pow(distToTarget, 2)
                - 2 * perpLength * distToTarget * Math.cos(camToPerpAngle));
        driveDist += Robot.drivetrain.getPosition(); // absolute

        // solve for angle to turn to drive on straight line to point perpendicular to target
        // uses law of sines, returns in degrees
        double angleC = Math.toDegrees(Math.asin((perpLength *
                Math.sin(camToPerpAngle)) / driveDist));
        double firstTurn = Robot.drivetrain.getGyroAngle() + limelightAngle + angleC;

        // return values as array
        return (new double[]{
            firstTurn,  // 0
            driveDist,  // 1
            targetAngle // 2
        });

>>>>>>> dac3939d77f91e09c190c6a08d125c91966dc38d
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
