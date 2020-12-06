/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.RobotStateDemacia;

public class AngleTest extends CommandBase {
  /**
   * Creates a new AngleTest.
   */
  Chassis chassis;
  double left, right;
  RobotStateDemacia state;
  int counter = 0;

  public AngleTest(Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    this.chassis = chassis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.left = SmartDashboard.getNumber("left (angle test)", 0);
    this.right = SmartDashboard.getNumber("right (angle test)", 0);
    System.out.println("left / right: " + left + " / " + right);
    state = new RobotStateDemacia(chassis);
    chassis.setVelocity2(left, right);
    counter = 0;
    // raceWith(new WaitCommand(2));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    if (counter == 100) {
      state = new RobotStateDemacia(chassis);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    state.compareAndPrint();
    chassis.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > 198;
  }
}
