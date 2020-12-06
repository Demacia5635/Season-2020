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


public class GoStraight extends CommandBase {
  public static final double KP = 1. / 60.;
  public static final double KI = 0; //KP / 10;
  public static final double KD = 0;

  private double velocity;
  private double distance;
  private double minDis = 0, maxDis = 0;
  private boolean stopAtEnd;
  private Chassis chassis;
  private RobotStateDemacia state;
  double angle;
  private double lastErr = 0;
  private double sumErr = 0;
  boolean withAngle = false;

  
  public GoStraight(double velocity, double distance, Chassis chassis, boolean stopAtEnd) {
    this.velocity = velocity;
    this.distance = distance;
    //this.velocity = SmartDashboard.getNumber("Velocity", 1);
    //this.distance = SmartDashboard.getNumber("Distance", 1);
    this.chassis = chassis;
    this.stopAtEnd = stopAtEnd;
    System.out.println("Go Stright -  with=" + withAngle);
    addRequirements(chassis);
  }

  public GoStraight(double velocity, double distance, double angle, Chassis chassis, boolean stopAtEnd){
    this(velocity, distance, chassis, stopAtEnd);
    this.angle = angle;
    withAngle = true;
    System.out.println("Go Stright - angle " + angle + " with=" + withAngle);
  }


  @Override
  public void initialize() {
    //this.velocity = SmartDashboard.getNumber("Velocity", 1);
    //this.distance = SmartDashboard.getNumber("Distance", 1);
    System.out.println("vel: " + velocity + "  distance: " + distance);
    state = new RobotStateDemacia(chassis);
    chassis.setVelocity2(velocity, velocity);
    minDis = chassis.getDistance() + distance - 0.1;
    maxDis = chassis.getDistance() + distance + 0.1;
    lastErr = 0;
    sumErr = 0;
  }

  @Override
  public void execute() {
    if(withAngle){
      double curAngle = chassis.getNormalizedAngle();
      double error = curAngle - angle;
      if(error > 180){
        error -= 360;
      }else if(error < -180){
        error += 360;
      }
      sumErr += error;
      double dE = error - lastErr;
      double fix = KP * error + KI * sumErr + KD * dE;
      double left = velocity + fix;
      double right = velocity - fix;
      lastErr = error;
      chassis.setVelocity2(left, right);
      System.out.println("normAng: " +  curAngle + " reqAngle: " + angle 
      + " error: " + error + " left / right: " + left + " / " + right);
    } 
  }

  @Override
  public void end(boolean interrupted) {
    if(stopAtEnd) {
      chassis.setPower(0, 0);
    }
    state.compareAndPrint();
  }

  @Override
  public boolean isFinished() {
    return chassis.getDistance() >= minDis && chassis.getDistance() <= maxDis;
  }
}
