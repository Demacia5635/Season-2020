/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.subsystems.Chassis;

public class RobotStateDemacia {
    double angle, leftDis, rightDis;
    Pose2d pose;
    long timeStep;
    Chassis chassis;

    public RobotStateDemacia(Chassis chassis){
        this.angle = chassis.getAngle();
        this.leftDis = chassis.getLeftDistance();
        this.rightDis = chassis.getRightDistance();
        this.pose = chassis.getPose();
        this.timeStep = System.currentTimeMillis();
        this.chassis = chassis;
    }

    public void compareAndPrint(){
        double dL = chassis.getLeftDistance() - leftDis;
        double dR = chassis.getRightDistance() - rightDis;
        double dt = (System.currentTimeMillis() - timeStep)/1000.;
        System.out.println("_______________________________");
        System.out.println("time : " + dt);
        System.out.println("angle : " + (chassis.getAngle() - angle));
        System.out.println("left distance : " + dL);
        System.out.println("left velocity : " + (dL / dt));
        System.out.println("right distance : " + dR);
        System.out.println("right velocity : " + (dR / dt));
        System.out.println("pose : " + chassis.getPose().minus(pose));
        System.out.println("_______________________________");

    }
}
