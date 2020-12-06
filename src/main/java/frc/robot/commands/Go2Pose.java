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

public class Go2Pose extends CommandBase {
  public static final double KP = 1. / 60.;
  public static final double KI = 0; // KP / 10;
  public static final double KD = 0;
  public static final double minR = 0.1;

  private double initAngle;
  private double velocity;
  private Pose2d pose;
  private double distance;
  private double angle;
  private boolean stopAtEnd;
  private Chassis chassis;
  private RobotStateDemacia state;
  private double lastErr = 0;
  private double sumErr = 0;
  boolean debug = false;

  public Go2Pose(double velocity, Translation2d translation, Chassis chassis, boolean stopAtEnd) {
    this.velocity = velocity;
    this.pose = new Pose2d(translation, new Rotation2d());
    // this.velocity = SmartDashboard.getNumber("Velocity", 1);
    // this.distance = SmartDashboard.getNumber("Distance", 1);
    this.chassis = chassis;
    this.stopAtEnd = stopAtEnd;
    initAngle = 0;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    // this.velocity = SmartDashboard.getNumber("Velocity", 1);
    // this.distance = SmartDashboard.getNumber("Distance", 1);
    Pose2d curr = chassis.getPose();
    
    state = new RobotStateDemacia(chassis);
    initAngle = chassis.getAngle2Pose(pose);
    System.out.println("vel: " + velocity + "  pose: " + pose + " curr: " + curr 
    + " angle: " + initAngle);
    lastErr = 0;
    sumErr = 0;
  }

  @Override
  public void execute() {
    double curAngle = chassis.getNormalizedAngle();
    angle = chassis.getAngle2Pose(pose);
    distance = chassis.getDistance2Pose(pose);
    double error = curAngle - angle;
    if (error > 180) {
      error -= 360;
    } else if (error < -180) {
      error += 360;
    }
    sumErr += error;
    double dE = error - lastErr;
    double fix = KP * error + KI * sumErr + KD * dE;
    double left = velocity + fix;
    double right = velocity - fix;
    lastErr = error;
    chassis.setVelocity2(left, right);
    if(debug) {
      System.out.println( "normAng: " + curAngle + " reqAngle: " 
      + angle + " left / right: " + left + " / " + right + " pose: " + chassis.getPose()
      + " dist: " + distance + " error: " + error + " angle: " + angle);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (stopAtEnd) {
      chassis.setPower(0, 0);
    }
    state.compareAndPrint();
  }

  @Override
  public boolean isFinished() {
    System.out.println("Angle-Init: " + chassis.diffAngle(angle, initAngle));
    return Math.abs(distance) < minR || Math.abs(chassis.diffAngle(angle, initAngle)) > 90;
  }
}
