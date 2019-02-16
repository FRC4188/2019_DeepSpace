/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * LED controll over I2C
 */
public class LED extends Subsystem {

  public static final int GREEN = 0;
  public static final int BLUE = 1;
  public static final int YELLOW = 2;
  public static final int RED = 3;
  public static final int RAINBOW = 4;

  public static final int SNAKE = 0;
  public static final int SOLID = 1;
  public static final int FADE = 2;

  public static I2C Wire = new I2C(Port.kOnboard, 8);

  /** Sets color of LEDs GREEN, BLUE, YELLOW, RED, | RAINBOW*/
  public void setColor(int color) {
    String command = Integer.toString(color);
    char[] commandC = command.toCharArray();
    byte[] commandB = new byte[commandC.length+1];
    byte[] inC = new byte[0];

    commandB[0] = (byte)'0'; // First int of sent data 0 so color is changed
    for(int i = 0; i < commandC.length; i++){
      commandB[i+1] = (byte)commandC[i];
    }

    Wire.transaction(commandB, commandB.length, inC, 0);
  }

  //Sets routine of LEDs SNAKE, SOLID, | FADE*/
  public void setRoutine(int routine) {
    String command = Integer.toString(routine);
    char[] commandC = command.toCharArray();
    byte[] commandB = new byte[commandC.length+1];
    byte[] inC = new byte[0];

    commandB[0] = (byte)'1'; // First int of sent data 1 so routine is changed
    for(int i = 0; i < commandC.length; i++){
      commandB[i+1] = (byte)commandC[i];
    }

    Wire.transaction(commandB, commandB.length, inC, 0);
  }

  /** Turns off LEDs */
  public void off(){
    char[] commandC = new char[] {'2'};
    byte[] commandB = new byte[commandC.length+1];
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
