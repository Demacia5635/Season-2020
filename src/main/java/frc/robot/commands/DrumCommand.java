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
import frc.robot.subsystems.Climb;

public class DrumCommand extends CommandBase {
  /**
   * Creates a new TelescopeCommand.
   */
  private XboxController controller;
  private Climb climb;
  private boolean isLocked = true;
  private long time = 0;

  public DrumCommand(Climb climb, XboxController controller) {
      this.climb = climb;
      this.controller = controller;
      addRequirements(climb.drumSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.lockLockPiston();
    isLocked = true;
    time = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Resets timer
    if (time != 0 && System.currentTimeMillis() > time + 500){
      time = 0;
      climb.turnOffLockPiston();
      climb.setDrumMotor(0);
    }
    if(climb.getInClimbing()){
      double power = controller.getY(Hand.kLeft);
      //Releases Lock piston when wanting to move
      if(power > 0.1 || power < -0.1){
        if(isLocked){
          isLocked = false;
          climb.releaseLockPiston();
          time = System.currentTimeMillis();
        }
        climb.setDrumMotor(power);
      }
      //Locks the piston when no power is given
      else{
        if(!isLocked){
          climb.lockLockPiston();
          isLocked = true;
          time = System.currentTimeMillis();
          climb.setDrumMotor(0.2);
        }
        if(time == 0){
          climb.setDrumMotor(0);
        }
      }
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
