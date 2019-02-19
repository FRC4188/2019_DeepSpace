package robot.utils;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Allows commands to be cycled and run by incrementing or
 *  decrementing a value using two Trigger objects. */
public class CommandIterator {

    private Trigger incrementer, decrementer;
    private ArrayList<Command> commandsList = new ArrayList<>();
    private List<Integer> valuesList = new ArrayList<>();
    private Command currentCommand = null;
    private int timer = 0;
    private int msBetweenClicks, counter, initialCounter, minCounter, maxCounter;
    private boolean lastIncrementer, lastDecrementer, firstPress = false;
    private String smartDashKey;

    /** Constructs new CommandIterator object to increment counter when incrementer is true, decrement
     *  counter when decrementer is true, and msBetweenClicks to wait for subsequent increments or decrements. */
    public CommandIterator(Trigger incrementer, Trigger decrementer, int msBetweenClicks, String smartDashKey) {
        this.incrementer = incrementer;
        this.decrementer = decrementer;
        this.msBetweenClicks = msBetweenClicks;
        this.smartDashKey = smartDashKey;
    }

    /** Assigns a given command to be run when counter reaches a given value. */
    public void runCmdWhenValue(Command command, int val) {
        if(valuesList.isEmpty()) minCounter = val;
        commandsList.add(command);
        valuesList.add(val);
        if(val > maxCounter) maxCounter = val;
        if(val < minCounter) minCounter = val;
        initialCounter = counter = minCounter - 1;
    }

    /** Returns current value of counter. */
    public double getCounter() {
        return counter;
    }

    /** Returns currently selected command. */
    public Command getCurrentCommand() {
        return currentCommand;
    }

    /** Begins incrementing counter and executing commands on correct values.
     *  Call this after all commands have been mapped to their counter values. */
    public void start() {
        Notifier notif = new Notifier(() -> {

            if(RobotState.isDisabled()) {
                counter = initialCounter;
                firstPress = false;
            } else {

                // get trigger vals
                boolean currentIncrementer = incrementer.get();
                boolean currentDecrementer = decrementer.get();

                // if button pressed then released, increment or decrement counter
                // and reset timer
                if(lastIncrementer) {
                    if(!currentIncrementer) {
                        counter++;
                        firstPress = true;
                    }
                    timer = 0;
                } else if(lastDecrementer && counter != initialCounter) {
                    if(!currentDecrementer) {
                        counter--;
                    }
                    timer = 0;
                }

                // constrain
                if(counter != initialCounter || firstPress) {
                    counter = (int) CSPMath.constrain(counter, minCounter, maxCounter);
                }

                //increase timer
                timer++;

                // if time exceeds paramater, save counter and run commands (20 ms per loop)
                if(timer > msBetweenClicks / 20 && firstPress) {
                    for(int i = 0; i < valuesList.size(); i++) {
                        if(counter == valuesList.get(i)) {
                            currentCommand = commandsList.get(i);
                            //currentCommand.start();
                            SmartDashboard.putString(smartDashKey, counter + ": " + currentCommand.toString());
                        }
                    }
                    timer = 0;
                }

                // save current button state for next loop
                lastIncrementer = incrementer.get();
                lastDecrementer = decrementer.get();

            }

        });
        notif.startPeriodic(0.02);
    }

}
