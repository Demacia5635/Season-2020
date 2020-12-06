/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class VelocityDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Chassis chassis;
  private XboxController controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VelocityDrive(Chassis subsystem, XboxController controller) {
    this.chassis = subsystem;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double inputSpeed = deadband(controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft), 0.05);
    double inputAngle = -deadband(controller.getX(Hand.kLeft), 0.05);
    if (chassis.inVisionMode(controller)) {
      // Vision Mode Code {
      chassis.driveToBall(inputSpeed);
    } else {
      // Main Speed Code
      double speed = Math.abs(inputSpeed);
      double angle = Math.abs(inputAngle);
      double maxSpeed = Constants.maxVel;
      speed = inputSpeed * (maxSpeed - angle * (maxSpeed - Constants.maxRotationV));
      angle = ((1 - Math.abs(inputSpeed)) * Constants.maxAngVel + 4) * inputAngle * (speed < 0 ? -1 : 1);
      System.out.println("angle: " + angle);
      chassis.setVelocityAndAngular(speed, angle);
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
