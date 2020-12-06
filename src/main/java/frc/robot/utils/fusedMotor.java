/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class  fusedMotor extends TalonSRX{
    public static final double STUCK_CURRENT = 20;
    private static final int STUCK_CYCLES = 125;
    private static final int PAUSE_CYCLES = 40 + STUCK_CYCLES;
    int stuckCycle;
    double sgn;
    String name;
    double outputValue;
    ControlMode controlMode;

    public fusedMotor(int id, String name) {
        super(id);
        this.name = name;
        stuckCycle = 0;
        sgn = 0;
        outputValue = 0;
        controlMode = ControlMode.PercentOutput;
        configContinuousCurrentLimit(20);
        configPeakCurrentLimit(20);
        enableCurrentLimit(true);
        Shuffleboard.getTab("tab").addNumber(name + ": current", this::getStatorCurrent);
        Shuffleboard.getTab("tab").addNumber(name + ": voltage", this::getMotorOutputVoltage);
    }

    @Override
    public void set(ControlMode mode, double outputValue) { 
        this.outputValue = outputValue;
        this.controlMode = mode;
        if(outputValue != 0){
        }
        double newSgn = Math.signum(outputValue);
        //fusing the motor when ball gets stuck
        if(newSgn != sgn){
            stuckCycle = 0;
            sgn = newSgn;
            super.set(mode, outputValue);
            return;
        }
        if(Math.abs(getStatorCurrent()) > STUCK_CURRENT){
            if(stuckCycle < STUCK_CYCLES){
                stuckCycle++;
                super.set(mode, outputValue);
            }else if(stuckCycle < PAUSE_CYCLES){
                //about pause_cycles/50 secs of not letting the robot collect any more balls 
                SmartDashboard.putBoolean("stop " + name + " motor", true);
                super.set(mode, 0);
                stuckCycle++;
                return;
            }else{
                stuckCycle = 0;
            }
        }else{
            SmartDashboard.putBoolean("stop " + name + " motor", false);
            stuckCycle = 0;
        }
        super.set(mode, outputValue);
    }

    public void reset(){
        set(controlMode, outputValue);
    }

}
