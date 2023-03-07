package frc.robot.controller;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;

public class Controller {
    private final Joystick joystick;

    public JoystickButton A, B, X, Y, leftBumper, rightBumper, leftStick, rightStick;
    public POVButton POVNorth, POVEast, POVSouth, POVWest; 

    public Controller(int joystickPort) {
        joystick = new Joystick(joystickPort);

        A = new JoystickButton(joystick, 1); // A Button
        B = new JoystickButton(joystick, 2); // B Button
        X = new JoystickButton(joystick, 3); // X Button
        Y = new JoystickButton(joystick, 4); // Y Button
        leftBumper = new JoystickButton(joystick, 5); // Left Bumper
        rightBumper = new JoystickButton(joystick, 6); // Right Bumper
        leftStick = new JoystickButton(joystick, 9); // Left Stick Button
        rightStick = new JoystickButton(joystick, 10); // Right Stick Button
    }

    private static void bindEvent(Consumer<Command> action, Command reaction){
        action.accept(reaction);
    }
    
    public static void onPress(Trigger action, Command reaction){
        bindEvent(action::onTrue, reaction);
    }

    public static void onHold(Trigger action, Command reaction){
        bindEvent(action::whileTrue, reaction);
    }

    public static void onRelease(Trigger action, Command reaction){
        bindEvent(action::onFalse, reaction);
    }

    public static void onPressToggle(Trigger action, Command reaction){
        bindEvent(action::toggleOnTrue, reaction);
    }

    public double getLeftStickX() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftXAxisID);
    }

    public double getLeftStickY() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftYAxisID);
    }

    public double getLeftTrigger() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickLeftTriggerID);
    }

    public double getRightTrigger() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickRightTriggerID);
    }

    public double getRightStickX() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickRightXAxisID); 
    }

    public double getRightStickY() {
        return joystick.getRawAxis(OIConstants.kPrimaryJoystickRightYAxisID);
    }
}
