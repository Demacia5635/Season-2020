/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class GroupOfMotors {
    private TalonSRX lead;
    private TalonSRX[] followers;
    
    public GroupOfMotors(int...talons){
        lead = new TalonSRX(talons[0]);
        lead.config_kP(0, Constants.isRobotA ? Constants.RobotA.kP : Constants.RobotB.kP);
        lead.config_kD(0, Constants.kD);
        lead.setNeutralMode(NeutralMode.Brake);
        lead.configContinuousCurrentLimit(Constants.maxCurrent);
        lead.enableCurrentLimit(true);
        followers = new TalonSRX[talons.length -1];
        for(int i = 0; i <followers.length; i++)
        {
            followers[i] = new TalonSRX(talons[i + 1]);
            followers[i].setNeutralMode(NeutralMode.Brake);
            followers[i].follow(lead);
        }
    }

    public void setPower(double power){ // -1 <= power <= 1
        lead.set(ControlMode.PercentOutput, power);
    }

    public void setRelVelocity(double vel, SimpleMotorFeedforward aff){// -1 <= vel <= 1
        setVelocity(vel * Constants.maxVel, aff);
    }

    
  public double getVelocity(){
        return lead.getSelectedSensorVelocity() / Constants.pulsePerMeter * 10;
  }

    public void resetEncoder(){
        lead.setSelectedSensorPosition(0);
    }

    public void setVelocity(double vel, SimpleMotorFeedforward aff){// m/s
        System.out.println("Speed: " + vel);
        double speed = (vel * Constants.pulsePerMeter / 10.);
        double a = (vel - getVelocity())*0.;
        double af = (aff.calculate(vel, a))/(12. / (5./5.));
        if(af != 0 || vel != 0){
        }
        lead.set(ControlMode.Velocity, speed, DemandType.ArbitraryFeedForward, af);
    }

    public void setVelocity(double vel, double aff){
        lead.set(ControlMode.Velocity, vel * Constants.pulsePerMeter / 10., DemandType.ArbitraryFeedForward, aff);
    }

    public void setVelocityWithAcceleration(double vel, SimpleMotorFeedforward aff){// m/s
        double speed = vel * Constants.pulsePerMeter / 10.;
        double a = (vel - getVelocity());
        double af = (aff.calculate(vel, a))/12.;
        lead.set(ControlMode.Velocity, speed, DemandType.ArbitraryFeedForward, af);
    }

    public double getAccelForSpeed(double vel) {
        return vel - getVelocity();
    }


    public void setK_P(double k_p){
        lead.config_kP(0, k_p);
    }

    public void setK_I(double k_i){
        lead.config_kI(0, k_i);
    }

    public void setK_D(double k_d){
        lead.config_kD(0, k_d);
    }

    public void invertMotors(){
        lead.setInverted(true);
        for(TalonSRX talon : followers){
            talon.setInverted(InvertType.FollowMaster);
        }
    }

    public double getEncoder(){
        return lead.getSelectedSensorPosition();
    }

    public double getDistance(){ //Meters
        return getEncoder()/Constants.pulsePerMeter;
    }

    public void invertEncoder(){
        lead.setSensorPhase(true);
    }
}
