package robot.subsystems;

import robot.Robot;
import robot.commands.vision.LimeLightDefault;
import robot.utils.CSPMath;
import robot.utils.PointFinder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;

import java.util.ArrayList;

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

    // flipping stuff
    private boolean isFlipped = false;
    private Servo flipServo;

    // camtran data
    ArrayList<double[]> camtranBuffer;

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
                BAY_CLOSE(3, 6.0/12), BAY_3D(4,6.0/12),
                CARGO_FLIP(5, 13.0/12), HATCH_FLIP(6, 19.0/12),
                BAY_CLOSE_FLIP(7, 6.0/12), BAY_3D_FLIP(4, 6.0 / 12);

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

    @Override
    public void periodic(){
        SmartDashboard.putNumber("limelightDistance", getDistance(currentPipeline.getHeight()));
        if(currentPipeline == Pipeline.BAY_3D || currentPipeline == Pipeline.BAY_3D_FLIP){
            double[] camtran = getCamtran();
            if (camtran != null) {
                camtranBuffer.add(camtran);
                if (camtranBuffer.size() > 5)
                    camtranBuffer.remove(0);
            }
        }
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

        flipServo = new Servo(0);
        camtranBuffer = new ArrayList<>();
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

    /**
     * Returns the 3d camera data
     */
    public double[] getCamtran(){
        return limelightTable.getEntry("camtran").getDoubleArray((double[])null);
    }

    /** Returns distance in feet from object of height s (feet). 
     *  Uses s = r(theta). */
    public double getDistance(double objectHeight) {
        final double CAMERA_HEIGHT = 720; // pixels
        final double CAMERA_FOV = Math.toRadians(49.7); // rads
        double boxHeight = limelightTable.getEntry("tvert").getDouble(0.0); // pixels
        if(boxHeight == 0) return 0;
        double percentHeight = boxHeight / CAMERA_HEIGHT;
        double boxDegree = percentHeight * CAMERA_FOV;
        double r = objectHeight / boxDegree; // feet
        return r; // fudge boy
    }

    /**
     * Returns distance in feet from object of width given
     */
    public double getDistance(double objectHeight, double boxHeight) {
        if(boxHeight == 0) return 0;
        final double CAMERA_HEIGHT = 720; // pixels
        final double CAMERA_FOV = Math.toRadians(49.7); // rads
        double percentHeight = boxHeight / CAMERA_HEIGHT;
        double boxDegree = percentHeight * CAMERA_FOV;
        double r = objectHeight / boxDegree; // feet
        return r; // fudge lad
    }

    /**
     * Returns the robot angle relative to the wall.
     */
    public double getRobotAngle(){
        ArrayList<Double> positives = new ArrayList<>();
        ArrayList<Double> negatives = new ArrayList<>();
        for(double[] camtran: camtranBuffer){
            if(camtran[4] >= 0){
                positives.add(camtran[4]);
            } else {
                negatives.add(camtran[4]);
            }
        }
        double[] pos = new double[positives.size()];
        double[] neg = new double[negatives.size()];
        for(int i=0;i<positives.size();++i){
            pos[i] = positives.get(i);
        }
        for (int i = 0; i < negatives.size(); ++i) {
            neg[i] = negatives.get(i);
        }
        SmartDashboard.putNumberArray("AnglePositive", pos);
        SmartDashboard.putNumberArray("AngleNegative", neg);
        if(positives.size() > negatives.size()){
            return CSPMath.average(positives);
        } else if(negatives.size() > positives.size()){
            return CSPMath.average(negatives);
        } else { 
            return 0.0;
        }
    }

    /**
     * Returns the distances using the 3d function in feet
     */
    public double[] getDistance3d(){
        double zOffset;
        double xOffset;
        ArrayList<Double> positives = new ArrayList<>();
        ArrayList<Double> negatives = new ArrayList<>();
        ArrayList<Double> zOffsets = new ArrayList<>();
        for (double[] camtran : camtranBuffer) {
            if (camtran[0] >= 0) {
                positives.add(camtran[0]);
            } else {
                negatives.add(camtran[0]);
            }
            zOffsets.add(camtran[2]);
        }
        if (positives.size() > negatives.size()) {
            xOffset = CSPMath.average(positives);
        } else if (negatives.size() > positives.size()) {
            xOffset = CSPMath.average(negatives);
        } else {
            xOffset = 0.0;
        }
        double[] pos = new double[positives.size()];
        double[] neg = new double[negatives.size()];
        double[] zed = new double[zOffsets.size()];
        for (int i = 0; i < positives.size(); ++i) {
            pos[i] = positives.get(i);
        }
        for (int i = 0; i < negatives.size(); ++i) {
            neg[i] = negatives.get(i);
        }
        for(int i=0;i<zOffsets.size();++i){
            zed[i] = zOffsets.get(i);
        }
        SmartDashboard.putNumberArray("Xpositive", pos);
        SmartDashboard.putNumberArray("Xnegative", neg);
        SmartDashboard.putNumberArray("Zoff", zed);
        zOffset = Math.abs(CSPMath.average(zOffsets));
        return new double[]{xOffset/12, zOffset/12};
    }

    /** Returns necessary distances and turns to get from current location to
     * line perpendicular to vision target, 4 ft away. Returns a double array
     * with the angle needed to turn to drive on line to point [0], the distance to 
     * drive to the point [1], and the angle to turn to face perpendicular to target
     * once distance has been driven [2]. Returned in units of feet and degrees. */
    public double[] solvePerpendicular() {

        // length away we want to be from target once perpendicular (ft)
        final double PERP_LENGTH = 3.5;
        // get known side lengths and angles (feet and degrees)
        // all angles relative to target, not field
        double robotAngle = getRobotAngle();
        double limelightAngle = getHorizontalAngle();
        //double distToTarget = getDistance(getPipeline().getHeight());
        double[] distances = getDistance3d();
        double sideDistance = distances[0];
        double forwardDistance = distances[1];
        double newForwardDistance = forwardDistance - PERP_LENGTH;
        
        // if the distance to the target is less than PERP_LENGTH, return all 0
        if (newForwardDistance < 0){
            return new double[]{0.0, 0.0, 0.0};
        }
        // if the side difference is negligable, just turn towards the target;
        if(Math.abs(sideDistance) < 1){
            return new double[]{limelightAngle, newForwardDistance, 0.0};
        }
        // if the side difference is negligable, just turn towards the target;
        if(Math.abs(sideDistance) < 1){
            return new double[]{limelightAngle, newForwardDistance, 0.0};
        }

        double firstTurn = Math.atan2(newForwardDistance, sideDistance) + robotAngle;
        double driveDistance = Math.sqrt(sideDistance * sideDistance + newForwardDistance * newForwardDistance);
        double secondTurn = 90 - Math.atan2(newForwardDistance, sideDistance);

        return new double[]{firstTurn, driveDistance, secondTurn};
    }

    /**
     * Flips the LimeLight so it faces the other way
     */
    public void flipCamera(){
        if(isFlipped){
            // unflip
            flipServo.setAngle(0.0);
            switch (currentPipeline) {
            case CARGO_FLIP:
                setPipeline(Pipeline.CARGO);
                break;
            case HATCH_FLIP:
                setPipeline(Pipeline.HATCH);
                break;
            case BAY_CLOSE_FLIP:
                setPipeline(Pipeline.BAY_CLOSE);
                break;
            case BAY_3D_FLIP:
                setPipeline(Pipeline.BAY_3D);
                break;
            default:
                // do nothing
                break;
            }
        } else{
            // flip
            flipServo.setAngle(360.0);
            switch (currentPipeline) {
            case CARGO:
                setPipeline(Pipeline.CARGO_FLIP);
                break;
            case HATCH:
                setPipeline(Pipeline.HATCH_FLIP);
                break;
            case BAY_CLOSE:
                setPipeline(Pipeline.BAY_CLOSE_FLIP);
                break;
            case BAY_3D:
                setPipeline(Pipeline.BAY_3D_FLIP);
                break;
            default:
                // do nothing
                break;
            }
        }
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
    public void trackBay() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        if(isFlipped){
            setPipeline(Pipeline.BAY_CLOSE_FLIP);
        } else {
            setPipeline(Pipeline.BAY_CLOSE);
        }
    }

    public void trackBay3D(){
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        if (isFlipped) {
            setPipeline(Pipeline.BAY_3D_FLIP);
        } else {
            setPipeline(Pipeline.BAY_3D);
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
        setPipeline(Pipeline.OFF);
    }
}
