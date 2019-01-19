package robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import robot.commands.vision.LimeLightUseAsCamera;

/** Limelight vision camera. Used to detect reflective tape. */
public class LimeLight extends Subsystem {

    // limelight network table
    NetworkTable limelightTable = null;

    // distance for target
    private double targetDistance = 0.0;
    private double targetHeight = 0.0;

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
        OFF(0), CARGO(1), HATCH(2), BAY(3);

        private final int value;
        Pipeline(int value) {
            this.value = value;
        }
        public int getValue() {
            return this.value;
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
     * @return the camera sees a target
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getBoolean(false);
    }

    /**
     * Returns the horizontal angle from the center of the camera to the target.
     * @return the horizontal angle to the target
     */
    public double getHorizontalAngle() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    /**
     * Returns the vertical angle from the center of the camera to the target.
     * @return the vertical angle to the target
     */
    public double getVerticalAngle() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    /**
     * Return the distance from the camera to the target.
     * @return the distance from the camera to the target
     */
    public double getDistance() {
        /*
        *Uses the equation: tan(a + ty) = (ht - hc) / d
        * a: the angle of the camera from the ground
        * ty: the measured angle of the target from the camera
        * ht: the height of the target
        * hc: the height of the camera
        * d: the distance
        */
        double a = 0.0; 
        double ty = getVerticalAngle();
        double ht = targetHeight;
        double hc = 1.25; // TODO modify for actual camera height
        return (ht - hc)/Math.tan(Math.toRadians(a + ty)) - targetDistance;
    }

    /**
     * Return the angle of the robot from the wall. 0 degrees means facing the wall.
     * @return the angle of the robot
     */
    public double getRobotAngle() {
        // ensure we are tracking bays
        if(currentPipeline != Pipeline.BAY) return 0;
        // get angles to raw contours (rough guess)
        double leftAngleX = limelightTable.getEntry("tx0").getDouble(0.0)*27;
        double rightAngleX = limelightTable.getEntry("tx1").getDouble(0.0)*27;
        double leftAngleY, rightAngleY;
        if(rightAngleX < leftAngleX) {
            double temp = leftAngleX;
            leftAngleX = rightAngleX;
            rightAngleX = temp;
            leftAngleY = limelightTable.getEntry("ty1").getDouble(0.0)*20.5;
            rightAngleY = limelightTable.getEntry("ty0").getDouble(0.0)*20.5;
        }else{
            leftAngleY = limelightTable.getEntry("ty0").getDouble(0.0)*20.5;
            rightAngleY = limelightTable.getEntry("ty1").getDouble(0.0)*20.5;
        }

        // calculate the distances to the two raw contours
        double a = 0.0;
        double ht = targetHeight;
        double hc = 1.25;
        double leftDistance = (ht-hc)/Math.tan(Math.toRadians(a+leftAngleY));
        double rightDistance = (ht-hc)/Math.tan(Math.toRadians(a+rightAngleY));

        // calculate the slope between the contours and get the angle
        double diffX = rightDistance*Math.cos(Math.toRadians(rightAngleX)) - leftDistance*Math.cos(Math.toRadians(leftAngleX));
        double diffY = rightDistance*Math.sin(Math.toRadians(rightAngleX)) - leftDistance*Math.sin(Math.toRadians(leftAngleX));
        return Math.toDegrees(Math.atan(diffX / diffY));
    }

    /**
     * Return the angle to turn to be 15 inches in front of the target.
     * @return the angle to turn
     */
    public double getTurnAngleToBay() {
        double distance = getDistance();
        double angle = getHorizontalAngle();
        double robotAngle = getRobotAngle();
        double horzDiff = distance * Math.sin(Math.toRadians(robotAngle + angle));
        double vertDiff = distance * Math.cos(Math.toRadians(robotAngle + angle)) - 15.0 / 12; // slightly inwards fromedge of tape
        if(vertDiff == 0) return 0;
        return robotAngle - Math.atan(horzDiff / vertDiff);
    }

    /**
     * Start tracking the ship bays
     */
    public void trackShipBay() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        setPipeline(Pipeline.BAY);
        targetDistance = 1.0;
        targetHeight = 3.05;
    }

    /**
     * Start tracking the rocket bays (slightly higher up)
     */
    public void trackRocketBay() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        setPipeline(Pipeline.BAY);
        targetDistance = 1.0;
        targetHeight = 3.76;
    }

    /**
     * Start tracking the cargo
     */
    public void trackCargo() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        setPipeline(Pipeline.CARGO);
        targetDistance = 0.5;
        targetHeight = 6.5 / 12; // 13 inch diameter, so look for center
    }

    /**
     * Start tracking the hatches
     */
    public void trackHatch() {
        setLightMode(LedMode.ON);
        setCameraMode(CameraMode.VISION);
        setPipeline(Pipeline.HATCH);
        targetDistance = 0.5;
        targetHeight = 0.0; // only track hatches when they are laying on the ground
    }

    /**
     * Use LimeLight as camera
     */
    public void useAsCamera() {
        setLightMode(LedMode.OFF);
        setCameraMode(CameraMode.CAMERA);
        setPipeline(Pipeline.BAY);
        targetDistance = 0.0;
        targetHeight = 0.0;
    }
}
