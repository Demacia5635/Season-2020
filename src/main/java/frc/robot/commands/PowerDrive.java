/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.RobotContainer.DrivingMethods;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.InputDev;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class PowerDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Chassis chassis;
  private InputDev inputDevice;

  public Command setInputDevice(InputDev device) {
    inputDevice = device;
    return this;
  }

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PowerDrive(Chassis subsystem, InputDev inputDevice) {
    this.chassis = subsystem;
    this.inputDevice = inputDevice;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.gyro.setFusedHeading(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    XboxController controller = inputDevice.getController();
    double inputPower = deadband(inputDevice.getValue1(), 0.1);
    double inputAngle = deadband(inputDevice.getValue2(), 0.1);

    if (chassis.inVisionMode(controller)) {
    //Vision Mode
      if (inputDevice.getDrivingMethod() == DrivingMethods.JS_2
        || inputDevice.getDrivingMethod() == DrivingMethods.XBOX_2Sticks){
          chassis.driveToBall((inputPower + inputAngle) / 2);
        }
      else{
        chassis.driveToBall(inputPower);
      }
    } else if (inputDevice.getDrivingMethod() == DrivingMethods.JS_2
        || inputDevice.getDrivingMethod() == DrivingMethods.XBOX_2Sticks) {
    // 2 Hands Code
      double left = deadband(inputDevice.getValue1(), 0.1);
      double right = deadband(inputDevice.getValue2(), 0.1);
      left = Math.abs(left) * left;
      right = Math.abs(right) * right;

      chassis.setPower(left, right);
    }
    else{
    // Main Power Code
    inputPower = Math.abs(Math.pow(inputPower, 2)) * inputPower;
    inputAngle = Math.abs(Math.pow(inputAngle, 2)) * inputAngle * (inputPower < -0.05 ? -1 : 1);

    double left = inputPower > 0 ? Math.min(inputPower - inputAngle, 1) : Math.max(inputPower - inputAngle, -1);
    double right = inputPower > 0 ? Math.min(inputPower + inputAngle, 1) : Math.max(inputPower + inputAngle, -1);

    chassis.setPower(left, right);

    if (inputPower != 0 || inputAngle != 0) {
    }
  }
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
