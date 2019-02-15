package robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * LED controll over I2C
 */
public class LED extends Subsystem {

    private static String command;
    private static char[] commandC;
    private static byte[] commandB;

    public static final int GREEN = 0;
    public static final int BLUE = 1;
    public static final int YELLOW = 2;
    public static final int RED = 3;
    public static final int RAINBOW = 4;

    public static final int SNAKE = 0;
    public static final int SOLID = 1;
    //public static final int FADE = 2;

    public static I2C Wire = new I2C(Port.kOnboard, 8);

    public void setColor(int color) {
        command = Integer.toString(color);
        commandC = command.toCharArray();
        commandB = new byte[commandC.length+1];
        byte[] inC = new byte[0];

        commandB[0] = 0; // First int of sent data 0 so color is changed
        for(int i = 0; i < commandC.length; i++){
            commandB[i+1] = (byte)commandC[i];
        }

        Wire.transaction(commandB, commandB.length, inC, 0);
    }

    public void setRoutine(int routine) {
        command = Integer.toString(routine);
        commandC = command.toCharArray();
        commandB = new byte[commandC.length+1];
        byte[] inC = new byte[0];

        commandB[0] = 1; // First int of sent data 1 so routine is changed
        for(int i = 0; i < commandC.length; i++){
          commandB[i+1] = (byte)commandC[i];
        }

        Wire.transaction(commandB, commandB.length, inC, 0);
    }

    public void off(){
        commandC = new char[] {'2'};
        commandB = new byte[commandC.length+1];
        byte[] inC = new byte[0];

        for(int i = 0; i < commandC.length; i++){
            commandB[i] = (byte)commandC[i];
        }

        Wire.transaction(commandB, commandB.length, inC, 0);
    }

    @Override
    public void initDefaultCommand() {
        setColor(LED.GREEN);
        setRoutine(LED.SNAKE);
    }

}
