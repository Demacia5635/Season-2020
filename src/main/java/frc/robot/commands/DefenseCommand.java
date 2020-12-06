/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.utils.PeriodicalSequentialCommandGroup;
import frc.robot.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DefenseCommand extends PeriodicalSequentialCommandGroup { //Always assuming the robot is reversed when it starts defending
  private static final double VELOCITY = 3;

/**
   * Creates a new DefenseCommand.
   */
  public DefenseCommand(Translation2d start, Translation2d end, Chassis chassis) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(chassis.getReverseRobotCommand(true)
    , new Go2Pose(VELOCITY, end, chassis
    , false).withTimeout(start.getDistance(end)/VELOCITY)
    , chassis.getReverseRobotCommand(false)
    , new Go2Pose(VELOCITY, start, chassis
    , false).withTimeout(end.getDistance(start)/VELOCITY));
  }
}
