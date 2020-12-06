/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotA;
import frc.robot.Constants.RobotB;
import frc.robot.utils.fusedMotor;


public class Balling extends SubsystemBase {
  /**
   * Creates a new BallsHandler.
   */

  public fusedMotor littleWheels;
  public fusedMotor bigWheels;
  public DoubleSolenoid ballsContainer;
  public DoubleSolenoid modePart1;
  public DoubleSolenoid modePart2;
  public boolean contain;

  public Balling() {
    if(!Constants.isRobotA){
      modePart1 = new DoubleSolenoid(RobotB.pcm1Port, RobotB.fMode1, RobotB.rMode1);
      modePart2 = new DoubleSolenoid(RobotB.pcm1Port, RobotB.fMode2, RobotB.rMode2);
      ballsContainer = new DoubleSolenoid(RobotB.pcm1Port, RobotB.fContainer, RobotB.rContainer);
    }else{
      modePart1 = new DoubleSolenoid(RobotA.pcmPort, RobotA.fMode1, RobotA.rMode1);
      modePart2 = new DoubleSolenoid(RobotA.pcmPort, RobotA.fMode2, RobotA.rMode2);
      ballsContainer = new DoubleSolenoid(RobotA.pcmPort, RobotA.fContainer, RobotA.rContainer);
    }
    littleWheels = new fusedMotor(Constants.lWheelsPort, "small");
    bigWheels = new fusedMotor(Constants.bWheelsPort, "big");
    contain = false;

  }

  public void moveLittleWheels(double power){
    if(Constants.isRobotA){
      littleWheels.set(ControlMode.PercentOutput, -power);
    }else{
      littleWheels.set(ControlMode.PercentOutput, power);
    }
  }
  public void collect(){    
    modePart1.set(Value.kForward);
    modePart2.set(Value.kForward);
    ballsContainer.set(Value.kForward);
    moveLittleWheels(-Constants.fastSmallW);
    bigWheels.set(ControlMode.PercentOutput, -Constants.slowBigW);
  }
  public void collecshoot(){    
    modePart1.set(Value.kForward);
    modePart2.set(Value.kForward);
    ballsContainer.set(Value.kForward);
    moveLittleWheels(-Constants.fastSmallW);
    bigWheels.set(ControlMode.PercentOutput, Constants.slowBigW);
  }
  public void subCollect(){
    ballsContainer.set(Value.kForward);
    moveLittleWheels(-Constants.fastSmallW);
    bigWheels.set(ControlMode.PercentOutput, -Constants.slowBigW);
  }
  public void shoot(){
    moveLittleWheels(-Constants.fastSmallW);
    bigWheels.set(ControlMode.PercentOutput, Constants.fastBigW);
  }
  public void slowShoot(){
    if(Constants.isRobotA){
      moveLittleWheels(-RobotA.superSlowSmallW);
      bigWheels.set(ControlMode.PercentOutput, RobotA.superSlowBigW);
    }else{
      moveLittleWheels(-RobotB.superSlowSmallW);
      bigWheels.set(ControlMode.PercentOutput, RobotB.superSlowBigW);
    }
  }
  public void unContain(){
    ballsContainer.set(Value.kReverse);
  }
  public void contain(){
    ballsContainer.set(Value.kForward);
  }
  public void release(){
    moveLittleWheels(Constants.slowSmallW);
  }
  public void offSolenoid(){
    ballsContainer.set(Value.kOff);
    modePart1.set(Value.kOff);
    modePart2.set(Value.kOff);
  }
  public void stopMotors(){
    littleWheels.set(ControlMode.PercentOutput, 0);
    bigWheels.set(ControlMode.PercentOutput, 0);
  }
  public void close(){
    System.out.println("closing");
    modePart1.set(Value.kReverse);
    modePart2.set(Value.kReverse);
  }
  public void middle(){
    System.out.println("middle");
    modePart1.set(Value.kReverse);
    modePart2.set(Value.kForward);
  }
  public void low(){
    System.out.println("lowing");
    modePart1.set(Value.kForward);
    modePart2.set(Value.kForward);
  }
  public CommandBase getShootingCommand(){
    return new InstantCommand(()->{
      shoot();
    });
  }
  public CommandBase getCollectingCommand(){
    return new InstantCommand(()->{
      collect();
    });
  }
  public CommandBase getCollecShootingCommand(){
    return new InstantCommand(()->{
      collecshoot();
    });
  }
  public CommandBase getStopMotorsCommand(){
    return new InstantCommand(()->{
      stopMotors();
    });
  }
  public CommandBase getLowCommand(){
    return new InstantCommand(()->{
      low();
    });
  }
  public CommandBase getMiddleCommand(){
    return new InstantCommand(()->{
      middle();
    });
  }
  public CommandBase getCloseCommand(){
    return new InstantCommand(()->{
      close();
    });
  }

  public CommandBase getSlowShootingCommand(){
    return new InstantCommand(() ->{
      slowShoot();
    });
  }

  public CommandBase getUnContainCommand(){
    return new InstantCommand(() ->{
      unContain();
    }, this);
  }

  public CommandBase getContainCommand(){
    return new InstantCommand(() ->{
      contain();
    }, this);
  }

  public CommandBase getThrow3BallsCommand(){
    return new InstantCommand(() ->{
      bigWheels.set(ControlMode.PercentOutput, 0.25);
      littleWheels.set(ControlMode.PercentOutput, 0.4
      );
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    bigWheels.reset();
    littleWheels.reset();
  }
}
