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

    // LED mode enum
    public enum ledMode{
        DEFAULT(0), OFF(1), BLINK(2), ON(3);

        private final int value;
        ledMode(int value){
            this.value = value;
        }
        public int getValue(){
            return this.value;
        }
    }

    // camera mode enum
    public enum cameraMode{
        VISION(0), CAMERA(1);

        private final int value;
        cameraMode(int value){
            this.value = value;
        }
        public int getValue(){
            return this.value;
        }
    }

    // pipeline enum
    public enum pipeline{
        BAY(0), CARGO(1), HATCH(2);

        private final int value;
        pipeline(int value){
            this.value = value;
        }
        public int getValue(){
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
    public LimeLight(){
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Sets the LED mode of the camera.
     * @param mode the LED mode to set the camera to
     */
    public void setLightMode(ledMode mode){
        limelightTable.getEntry("ledMode").setNumber(mode.getValue());
    }

    /**
     * Sets the camera mode of the camera.
     * @param mode the camera mode to set the camera to
     */
    public void setCameraMode(cameraMode mode){
        limelightTable.getEntry("camMode").setNumber(mode.getValue());
    }

    /**
     * Sets the pipeline for the camera to use.
     * @param pl the pipeline for the camera to use
     */
    public void setPipeline(pipeline pl){
        limelightTable.getEntry("pipeline").setNumber(pl.getValue());
    }

    /**
     * Returns if the camera sees a target.
     * @return the camera sees a target
     */
    public boolean hasTarget(){
        return limelightTable.getEntry("tv").getBoolean(false);
    }

    /**
     * Returns the horizontal angle from the center of the camera to the target.
     * @return the horizontal angle to the target
     */
    public double getHorizontalAngle(){
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    /**
     * Returns the vertical angle from the center of the camera to the target.
     * @return the vertical angle to the target
     */
    public double getVerticalAngle(){
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    /**
     * Return the distance from the camera to the target.
     * @return the distance from the camera to the target
     */
    public double getDistance(){
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
        double ht = 0.0;
        double hc = 0.0;
        return (ht - hc)/Math.tan(Math.toRadians(a + ty)) - targetDistance;
    }

    /**
     * Start tracking the ship bays
     */
    public void trackShipBay(){
        setLightMode(ledMode.ON);
        setCameraMode(cameraMode.VISION);
        setPipeline(pipeline.BAY);
        targetDistance = 1.0;
    }

    /**
     * Start tracking the rocket bays (slightly higher up)
     */
    public void trackRocketBay(){
        setLightMode(ledMode.ON);
        setCameraMode(cameraMode.VISION);
        setPipeline(pipeline.BAY);
        targetDistance = 1.5;
    }

    /**
     * Start tracking the cargo
     */
    public void trackCargo(){
        setLightMode(ledMode.ON);
        setCameraMode(cameraMode.VISION);
        setPipeline(pipeline.CARGO);
        targetDistance = 0.5;
    }

    /**
     * Start tracking the hatches
     */
    public void trackHatch(){
        setLightMode(ledMode.ON);
        setCameraMode(cameraMode.VISION);
        setPipeline(pipeline.HATCH);
        targetDistance = 0.5;
    }

    /**
     * Use LimeLight as camera
     */
    public void useAsCamera(){
        setLightMode(ledMode.OFF);
        setCameraMode(cameraMode.CAMERA);
        setPipeline(pipeline.BAY);
        targetDistance = 0.0;
    }

}
