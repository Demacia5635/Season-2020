/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.RobotStateDemacia;

public class TurnByDegrees extends CommandBase {

  public double speed;

  private double reqAngle;
  private double degrees;
  private final Chassis chassis;
  private final boolean withPID;
  private final boolean relative;
  private Pose2d pose;
  private double power;
  private RobotStateDemacia state;
  private static final double kP = 1/60;
  private static final double kI = kP/10;
  private static final double kD = 0;
  private double sumOfErrors;
  private double lastError;

  public TurnByDegrees(final double degrees, final Chassis chassis, final boolean withPID, boolean relative) {
    this.chassis = chassis;
    this.degrees = degrees;
    this.withPID = withPID;
    this.relative = relative;
    speed = 2.5;
    pose = null;
    addRequirements(chassis);
  }

  //turn2Pose
  public TurnByDegrees(final Translation2d translation, final Chassis chassis) {
    this(translation, 2, chassis);
  }

  public TurnByDegrees(final Translation2d translation, final double speed, final Chassis chassis) {
    this.chassis = chassis;
    this.degrees = 0;
    this.pose = new Pose2d(translation, new Rotation2d());
    this.speed = speed;
    relative = true;
    withPID = false;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    System.out.println("originals: degree = " + degrees  + " cur = " 
    + chassis.getNormalizedAngle() +  " rel = " + relative);
    state = new RobotStateDemacia(chassis);
    if(!relative){
      reqAngle = degrees + chassis.getNormalizedAngle();
    }else{
      double curAngle = chassis.getNormalizedAngle();
      if(pose == null){
        reqAngle = degrees;
      }else{
        reqAngle = chassis.getAngle2Pose(pose);
      }
      degrees = chassis.diffAngle(reqAngle, curAngle);

    }
    double left = -Math.signum(degrees) * speed;
    double right = -left;
    if(!withPID)
      chassis.setVelocity2(left,right);
    System.out.println("vel2 (left): " + left
                       + " reqAngle: " + reqAngle + " degrees: " + degrees);
    sumOfErrors = 0;
    lastError = 0;
  }

  @Override
  public void execute() {
    if (withPID){
      final double error = chassis.diffAngle(reqAngle, chassis.getNormalizedAngle());
      final double diffOfErrors = error - lastError;
      sumOfErrors += error;
      power = error * kP + sumOfErrors * kI + diffOfErrors * kD;
      double left = power*-Math.signum(degrees);
      chassis.setPower(left, -left);
      System.out.println("Turn PID, Left = " + left);
      lastError = error;
    } else {
      double left = -Math.signum(degrees) * speed;
      chassis.setVelocity2(left,-left);
      System.out.println("Turn, Left = " + left);
    }
  }

  @Override
  public void end(final boolean interrupted) {
    chassis.setVelocity(0, 0);
    state.compareAndPrint();
    System.out.println("Finished Turning");
  }

  @Override
  public boolean isFinished() {
    return Math.abs(chassis.diffAngle(chassis.getAngle(), reqAngle)) < 20;
  }
}
