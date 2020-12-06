/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.RobotStateDemacia;

public class turnWhileMoving extends CommandBase {
  /**
   * Creates a new turnByDistance.
   */
  public static final double R = 1;
  public static final double velOuter = 4;
  public static final double velInner = 0.4;
  double angle, startAngle , lastSpeed;
  Chassis chassis;
  RobotStateDemacia state;
  
  public turnWhileMoving(double angle,double lastSpeed, Chassis chassis) {
    // -180 < angle < 180
    addRequirements(chassis);
    this.angle = angle;
    this.lastSpeed = lastSpeed;
    this.chassis = chassis;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = new RobotStateDemacia(chassis);
    startAngle = chassis.getAngle();
    if(angle > 0){
      chassis.setVelocityWithAcceleration(velInner, velOuter);
    }else if(angle < 0){
      chassis.setVelocityWithAcceleration(velOuter, velInner);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    state.compareAndPrint();
    chassis.setVelocityWithAcceleration(lastSpeed, lastSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angle - chassis.getAngle() + startAngle) < 3;
  }
}
