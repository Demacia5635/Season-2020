/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Chassis;

/**
 * Add your docs here.
 */
public class RamseteCommandDemacia extends RamseteCommand {

    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final RamseteController m_follower;
    private final DifferentialDriveKinematics m_kinematics;
    private final BiConsumer<Double, Double> m_output;
    private Chassis chassis;

    public RamseteCommandDemacia(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController follower,
            DifferentialDriveKinematics kinematics, BiConsumer<Double, Double> outputMetersPerSecond,
            Chassis chassis) {
        super(trajectory, pose, follower, kinematics, outputMetersPerSecond, chassis);
        m_trajectory = trajectory;
        m_pose = pose;
        m_follower = follower;
        m_kinematics = kinematics;
        m_output = outputMetersPerSecond;
        this.chassis = chassis;
    }

    @Override
  public void initialize() {
    chassis.resetOdometry(m_trajectory.getInitialPose());
    super.initialize();
    m_timer.reset();
    m_timer.start();
  }

    @Override
  public void execute() {
    double curTime = m_timer.get();

    Pose2d cur = m_pose.get();
    State req = m_trajectory.sample(curTime);

    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
        m_follower.calculate(cur, req));
    System.out.println("trajectory\n" + req);
    System.out.println("odometry\n" + cur);

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    m_output.accept(leftSpeedSetpoint, rightSpeedSetpoint);

  }


  @Override
  public void end(boolean interrupted) {
    chassis.setVelocity(0, 0);
  }
}