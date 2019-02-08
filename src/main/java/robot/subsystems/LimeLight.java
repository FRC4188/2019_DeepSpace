package robot.subsystems;

import robot.Robot;
import robot.commands.vision.LimeLightDefault;
import robot.utils.CSPMath;
import robot.utils.PointFinder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import org.opencv.calib3d.*;
import org.opencv.core.*;

/** Limelight vision camera. Used to detect reflective tape. */
public class LimeLight extends Subsystem {

    // limelight network table
    NetworkTable limelightTable = null;

    // current pipeline
    private Pipeline currentPipeline = Pipeline.OFF;

    // matrix stuff
    private MatOfPoint3f mObjectPoints;
    private Mat mCameraMatrix;
    private MatOfDouble mDistortionCoefficients;

    // flipped status
    private boolean isFlipped = false;

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
                BAY_CLOSE(3, 6.0/12), BAY_HIGH(4, 6.0/12),
                CARGO_FLIP(5, 13.0/12), HATCH_FLIP(6, 19.0/12),
                BAY_CLOSE_FLIP(7, 6.0/12), BAY_HIGH_FLIP(8, 6.0/12);

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
        setDefaultCommand(new LimeLightDefault());
    }

    /**
     * Constructor for Limelight.
     */
    public LimeLight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        mObjectPoints = new MatOfPoint3f(new Point3(0.0, 0.0, 0.0), // bottom right
                new Point3(-1.9363, 0.5008, 0.0), // bottom left
                new Point3(-0.5593, 5.8258, 0.0), // top-left
                new Point3(1.377, 5.325, 0.0) // top-right
        );
        mCameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
        mCameraMatrix.put(0, 0, 2.5751292067328632e+02);
        mCameraMatrix.put(0, 2, 1.5971077914723165e+02);
        mCameraMatrix.put(1, 1, 2.5635071715912881e+02);
        mCameraMatrix.put(1, 2, 1.1971433393615548e+02);

        mDistortionCoefficients = new MatOfDouble(2.9684613693070039e-01, -1.4380252254747885e+00,
                -2.2098421479494509e-03, -3.3894563533907176e-03, 2.5344430354806740e+00);
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
        // get corner data
        double[] cornX = limelightTable.getEntry("tcornx").getDoubleArray(new double[0]);
        double[] cornY = limelightTable.getEntry("tcorny").getDoubleArray(new double[0]);
        if (cornX.length != 4 || cornY.length != 4) {
            System.out.println("[ERROR] Could not find 4 points from image");
            return 0.0;
        }
        // find the corners
        PointFinder pointFinder = new PointFinder(cornX, cornY);
        MatOfPoint2f imagePoints = new MatOfPoint2f(pointFinder.getBottomRight(), pointFinder.getBottomLeft(),
                pointFinder.getTopLeft(), pointFinder.getTopRight());
        Mat rotationVector = new Mat();
        Mat translationVector = new Mat();
        Calib3d.solvePnP(mObjectPoints, imagePoints, mCameraMatrix, mDistortionCoefficients, rotationVector,
                translationVector);
        Mat rotationMatrix = new Mat();
        Calib3d.Rodrigues(rotationVector, rotationMatrix);
        
        Mat projectionMatrix = new Mat(3, 4, CvType.CV_64F);
        projectionMatrix.put(0, 0, rotationMatrix.get(0, 0)[0], rotationMatrix.get(0, 1)[0],
                rotationMatrix.get(0, 2)[0], translationVector.get(0, 0)[0], rotationMatrix.get(1, 0)[0],
                rotationMatrix.get(1, 1)[0], rotationMatrix.get(1, 2)[0], translationVector.get(1, 0)[0],
                rotationMatrix.get(2, 0)[0], rotationMatrix.get(2, 1)[0], rotationMatrix.get(2, 2)[0],
                translationVector.get(2, 0)[0]);

        Mat cameraMatrix = new Mat();
        Mat rotMatrix = new Mat();
        Mat transVect = new Mat();
        Mat rotMatrixX = new Mat();
        Mat rotMatrixY = new Mat();
        Mat rotMatrixZ = new Mat();
        Mat eulerAngles = new Mat();
        Calib3d.decomposeProjectionMatrix(projectionMatrix, cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY,
                rotMatrixZ, eulerAngles);
        return eulerAngles.get(1, 0)[0]; 
    }

    /** Returns necessary distances and turns to get from current location to
     * line perpendicular to vision target, 4 ft away. Returns a double array
     * with the angle needed to turn to drive on line to point [0], the distance to 
     * drive to the point [1], and the angle to turn to face perpendicular to target
     * once distance has been driven [2]. Returned in units of feet and degrees. */
    public double[] solvePerpendicular() {

        // length away we want to be from target once perpendicular (ft)
        final double PERP_LENGTH = 4;

        // estimate field relative target angle based off current heading
        // currently works for all targets except two on end of ship
        // NEEDS WORK
        double targetAngle;
        double gyroAngle = Robot.drivetrain.getGyroAngle();
        if(CSPMath.isBetween(gyroAngle, 0, 39)) targetAngle = 28.75;
        else if(CSPMath.isBetween(gyroAngle, 40, 140)) targetAngle = 90;
        else if(CSPMath.isBetween(gyroAngle, 141, 180)) targetAngle = 151.25;
        else if(CSPMath.isBetween(gyroAngle, -1, -39)) targetAngle = -28.75;
        else if(CSPMath.isBetween(gyroAngle, -40, -140)) targetAngle = -90;
        else if(CSPMath.isBetween(gyroAngle, -141, -180)) targetAngle = -151.25;
        else targetAngle = 0;

        // get known side lengths and angles (feet and degrees)
        // all angles relative to target, not field
        double robotAngle = Robot.drivetrain.getGyroAngle() - targetAngle;
        double limelightAngle = getHorizontalAngle();
        double distToTarget = getDistance(getPipeline().getHeight());

        // angle between line from camera to target and perpendicular line in radians
        // found using parallel lines
        double camToPerpAngle = Math.toRadians(robotAngle - limelightAngle);

        // solve for distance to point perpendicular to target, PERP_LENGTH away
        // uses law of cosines
        double driveDist = Math.sqrt(Math.pow(PERP_LENGTH, 2) + Math.pow(distToTarget, 2)
                - 2 * PERP_LENGTH * distToTarget * Math.cos(camToPerpAngle));

        // solve for angle to turn to drive on straight line to point perpendicular to target
        // uses law of sines, returns in degrees
        double angleC = Math.toDegrees(Math.asin((PERP_LENGTH *
                Math.sin(camToPerpAngle)) / driveDist));
        double firstTurn = Robot.drivetrain.getGyroAngle() + limelightAngle + angleC;

        // if already almost perpendicular (5 deg tolerance) then return 0, else return vals
        if(Math.abs(camToPerpAngle) < Math.toRadians(5.0)) {
            return (new double[] { 0, 0, 0 });
        } else {
            return (new double[] {
                firstTurn,  // 0
                driveDist,  // 1
                targetAngle // 2
            });
        }

    }

    /**
     * Flips the LimeLight so it faces the other way
     */
    public void flipCamera(){
        isFlipped = !isFlipped;
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
        if(isFlipped){
            setPipeline(Pipeline.BAY_CLOSE_FLIP);
        } else {
            setPipeline(Pipeline.BAY_CLOSE);
        }
    }

    /**
     * Start tracking the closest rocket bays (slightly higher up)
     */
    public void trackRocketBayClose() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        if(isFlipped){
            setPipeline(Pipeline.BAY_CLOSE_FLIP);
        } else {
            setPipeline(Pipeline.BAY_CLOSE);
        }
    }

    /**
     * Start tracking the hightest rocket bays (slightly higher up)
     */
    public void trackRocketBayHigh() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        if(isFlipped){
            setPipeline(Pipeline.BAY_HIGH_FLIP);
        } else {
            setPipeline(Pipeline.BAY_HIGH);
        }
    }

    /**
     * Start tracking the cargo
     */
    public void trackCargo() {
        setLightMode(LedMode.OFF);
        setCameraMode(CameraMode.VISION);
        if(isFlipped){
            setPipeline(Pipeline.CARGO_FLIP);
        } else {
            setPipeline(Pipeline.CARGO);
        }
    }

    /**
     * Start tracking the hatches
     */
    public void trackHatch() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        if (isFlipped) {
            setPipeline(Pipeline.HATCH_FLIP);
        } else {
            setPipeline(Pipeline.HATCH);
        }
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
