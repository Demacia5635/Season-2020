/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer.DrivingMethods;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class InputDev {
    private DrivingMethods drivingMethod;
    private Joystick joystick;
    private Joystick leftJS;
    private Joystick rightJS;
    private XboxController controller;

    public DrivingMethods getDrivingMethod() {
        return this.drivingMethod;
    }

    public XboxController getController() {
        return controller;
    }

    public InputDev(Joystick leftJS, Joystick rightJS) {
        this.leftJS = leftJS;
        this.rightJS = rightJS;
        drivingMethod = DrivingMethods.JS_2;
    }

    public InputDev(int leftPort, int rightPort) {
        this.leftJS = new Joystick(leftPort);
        drivingMethod = DrivingMethods.JS_2;
    }

    public InputDev(Joystick joystick) {
        this.joystick = joystick;
        drivingMethod = DrivingMethods.JS_1;
    }

    public InputDev(XboxController controller, boolean is1Stick, boolean isOrbitMode) {
        this.controller = controller;
        drivingMethod = isOrbitMode ? DrivingMethods.XBOX_Orbit
                : is1Stick ? DrivingMethods.XBOX_1Stick : DrivingMethods.XBOX_2Sticks;
    }

    public InputDev(XboxController controller, DrivingMethods methods) {
        this.controller = controller;
        this.drivingMethod = methods;
    }

    public InputDev(int port, boolean isXbox, boolean is1JS, boolean isOrbitMode) {
        if (isXbox) {
            this.controller = new XboxController(port);
        } else {
            this.joystick = new Joystick(port);
        }
        drivingMethod = isXbox ? is1JS ? DrivingMethods.XBOX_1Stick : DrivingMethods.XBOX_2Sticks : DrivingMethods.JS_1;
    }

    public double[] getBothValues() {
        return new double[] { getValue1(), getValue2() };
    }

    // In case of 2 JS - Returns left y
    // In case of 1 JS - Returns y
    public double getValue1() {
        switch (drivingMethod) {
        case JS_1:
            return joystick.getY();
        case JS_2:
            return leftJS.getY();
        case XBOX_2Sticks:
            return controller.getY(Hand.kLeft);
        case XBOX_1Stick:
            return controller.getY(Hand.kLeft);
        case XBOX_Orbit:
            return controller.getY(Hand.kLeft);
        case YorgenMode:
            return controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
        case Einziger:
            return controller.getY(Hand.kLeft);
        default:
            return 0;
        }
    }

    // In case of 2 JS - Returns right y
    // In case of 1 JS - Returns x
    public double getValue2() {
        switch (drivingMethod) {
        case JS_1:
            return joystick.getX();
        case JS_2:
            return rightJS.getY();
        case XBOX_2Sticks:
            return controller.getY(Hand.kRight);
        case XBOX_1Stick:
            return controller.getX(Hand.kLeft);
        case XBOX_Orbit:
            return controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
        case YorgenMode:
            return -controller.getX(Hand.kLeft);
        case Einziger:
            return controller.getX(Hand.kRight);
        default:
            return 0;
        }
    }
}