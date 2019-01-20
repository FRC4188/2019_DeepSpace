package robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Arm extends Subsystem {

    /*

    // Device initialization
    private WPI_TalonSRX arm = new WPI_TalonSRX(0);
    private WPI_TalonSRX armSlave = new WPI_TalonSRX(0);
    private WPI_TalonSRX wrist = new WPI_TalonSRX(0);
    private WPI_TalonSRX wristSlave = new WPI_TalonSRX(0);

    */

    /** Constructs new arm object and configures devices. */

    /*
    public Arm() {

        // Slave control 
        armSlave.follow(arm);
        wristSlave.follow(wrist);

        // Encoders
        arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    }

    */
    @Override
    public void initDefaultCommand() {
        // setDefaultCommand(new MySpecialCommand());
    }

}
