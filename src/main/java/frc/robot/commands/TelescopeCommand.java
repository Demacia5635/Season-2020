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

public class TelescopeCommand extends CommandBase {
  /**
   * Creates a new TelescopeCommand.
   */
  private XboxController controller;
  private Climb climb;

  public TelescopeCommand(Climb climb, XboxController controller) {
      this.climb = climb;
      this.controller = controller;
      addRequirements(climb.telescopeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climb.getInClimbing()){
      double power = controller.getY(Hand.kRight);
      double povAngle = controller.getPOV();
      //If the pov is pressed up - moves up, if the pov is pressed down - moves down, else doesn't move
      double povPower = povAngle == 0 ? 0.5 : povAngle == 180 ? -0.5 : 0;
      //Happens only if going up and not in upper limit
      if(power < -0.1 && !climb.isUpperTeleLimit()){
        climb.setTelescopicMotor(power);
      }
      //Happens only if going down and not in lower limit
      else if(power > 0.1 && !climb.isLowerTeleLimit()){
        climb.setTelescopicMotor(power / 2);
      }
      //Happens when you wanna stop (no power / in upper limit and going up / in lower limit and going down)
      else {
        climb.setTelescopicMotor(povPower);
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
