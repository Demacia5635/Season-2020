/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer.DrivingMethods;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.InputDev;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class InputDeviceVelocityDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Chassis chassis;
  private InputDev inputDevice;

  public void setInputDevice(InputDev device) {
    inputDevice = device;
  }

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public InputDeviceVelocityDrive(Chassis subsystem, InputDev inputDevice) {
    this.chassis = subsystem;
    this.inputDevice = inputDevice;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.resetEncoders();
    chassis.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double inputSpeed = deadband(inputDevice.getValue1(), 0.1);
    double inputAngle = deadband(inputDevice.getValue2(), 0.1);
    XboxController controller = inputDevice.getController();
    if (chassis.inVisionMode(controller)) {
      // Vision Mode Code
      if (inputDevice.getDrivingMethod() == DrivingMethods.JS_2
          || inputDevice.getDrivingMethod() == DrivingMethods.XBOX_2Sticks) {
        chassis.driveToBall((inputSpeed + inputAngle) / 2);
      } else {
        chassis.driveToBall(inputSpeed);
      }
    } else if (inputDevice.getDrivingMethod() == DrivingMethods.JS_2
        || inputDevice.getDrivingMethod() == DrivingMethods.XBOX_2Sticks) {
      //2 JS Code
      double left = deadband(inputDevice.getValue1(), 0.1);
      double right = deadband(inputDevice.getValue2(), 0.1);
      left = Math.abs(left) * left;
      right = Math.abs(right) * right;
      chassis.setRelVelocity(left, right);
    } else {
      // Main Speed Code
      double speed = Math.abs(inputSpeed);
      double angle = Math.abs(inputAngle);
      double maxSpeed = SmartDashboard.getNumber("MaxSpeed(m/s)", 3);
      speed = inputSpeed * (maxSpeed - angle * (maxSpeed - Constants.maxRotationV));
      angle = ((1 - Math.abs(inputSpeed)) * Constants.maxAngVel + 4) * inputAngle * (speed < 0 ? -1 : 1);
      chassis.setVelocityAndAngular(speed, angle);
    }
    /*
     * speed = Math.abs(Math.pow(speed, 2)) * speed; angle =
     * Math.abs(Math.pow(angle, 2)) * angle * (speed < -0.05 ? -1 : 1);
     * 
     * left = speed > 0 ? Math.min(speed + angle, 1) : Math.max(speed + angle, -1);
     * right = speed > 0 ? Math.min(speed - angle, 1) : Math.max(speed - angle, -1);
     */
    /*
     * if(right == left && right != 0){ double error = chassis.getAngle() -
     * angleStart; double diffOfErrors = error - lastError; sumOfErrors += error;
     * double fix = kP*error + kI*sumOfErrors + kD*diffOfErrors; left += fix; right
     * -= fix; lastError = error; } else { angleStart = chassis.getAngle();
     * sumOfErrors = 0; lastError = 0; }
     */
    // System.out.println("Left " + left + " / " + "Right " + right);
    // System.out.println("Angle" + angle);
    // System.out.printf(" Left = %.2f , right = %.2f , angle = %3.1f\n", left,
    // right, angle);
  }

  private double deadband(double num, double min) {
    if (Math.abs(num) > min) {
      return num;
    } else {
      return 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
