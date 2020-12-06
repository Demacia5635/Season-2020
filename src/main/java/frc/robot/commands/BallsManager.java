/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Balling;

public class BallsManager extends CommandBase {
  /**
   * Creates a new BallsManager.
   */
  private enum Action {
    throwing, colloecting, normal, release, collecShoot, slowShoot;
  }

  private JoystickButton aButton;
  private JoystickButton lButton;
  private JoystickButton mButton;
  private JoystickButton hButton;
  private JoystickButton slowShootingButton;
  private JoystickButton climbButton;
  private JoystickButton collecShootButton;
  private JoystickButton unContainShootButton;
  private XboxController subController;
  private JoystickButton visionButton;
  private int actionCounter;
  private Balling balls;
  private Action mode = Action.normal;

  public BallsManager(Balling balls, XboxController driver, XboxController subDriver) {
    this.balls = balls;
    aButton = new JoystickButton(driver, 1);
    lButton = new JoystickButton(subDriver, 1);
    mButton = new JoystickButton(subDriver, 4);
    hButton = new JoystickButton(subDriver, 3);
    slowShootingButton = new JoystickButton(subDriver, 5);
    collecShootButton = new JoystickButton(driver, 3);
    unContainShootButton = new JoystickButton(subDriver, 6);
    climbButton = new JoystickButton(subDriver, 8);
    visionButton = new JoystickButton(driver, 4);
    subController = subDriver;
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balls.stopMotors();
    balls.close();
    balls.contain();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    actionCounter++;
    switch (mode){
      case normal:
        if(actionCounter==20){
          balls.offSolenoid();
        }
        if(mButton.get()){
          balls.middle();
          //balls.offSolenoid();
        }
        else if(lButton.get()){
          balls.low();
          //balls.offSolenoid();
        }
        else if(hButton.get()){
          balls.close();
          //balls.offSolenoid();
        }
        
        if(aButton.get() || visionButton.get()){
          mode = Action.colloecting;
          balls.collect();
          actionCounter=0;
        }
        else if(collecShootButton.get()){
          mode = Action.collecShoot;
          balls.collecshoot();
          actionCounter = 0;
        }
        else if(subController.getPOV()>130&&subController.getPOV()<230){
          mode = Action.release;
          balls.release();
          actionCounter=0;
        }
        else if(subController.getTriggerAxis(Hand.kLeft)>0.3){
          mode = Action.throwing;
          balls.shoot();
          actionCounter=0;
        }
        else if(subController.getTriggerAxis(Hand.kRight)>0.3){
          mode = Action.colloecting;
          balls.subCollect();
          actionCounter=0;
        }
        else if(unContainShootButton.get()){
          balls.unContain();
          balls.shoot();
          mode = Action.collecShoot;
          actionCounter = 0;
        } else if(slowShootingButton.get()){
          balls.slowShoot();
          mode = Action.slowShoot;
          actionCounter =0;
        }
        break;
      case throwing:
        if(actionCounter==50){
          balls.unContain();
        }
        if(actionCounter>=70){
          mode = Action.normal;
          actionCounter=0;
          balls.stopMotors();
        }
        break;
      case colloecting:
        if(!aButton.get()&&!(subController.getTriggerAxis(Hand.kRight)>0.3)){
          mode = Action.normal;
          actionCounter=0;
          balls.stopMotors();
        }
        break;
      case release:
        if(!(subController.getPOV()>130&&subController.getPOV()<230)){
          mode = Action.normal;
          actionCounter=0;
          balls.stopMotors();
        }
        break;
      case collecShoot:
        if(!(collecShootButton.get())&&!unContainShootButton.get()){
          mode = Action.normal;
          actionCounter = 0;
          balls.stopMotors();
        }
        break;
      case slowShoot:
        if(actionCounter == 35){
          balls.unContain();
        }
        if(actionCounter>=50){
          mode = Action.normal;
          actionCounter=0;
          balls.stopMotors();
        }
        break;
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
