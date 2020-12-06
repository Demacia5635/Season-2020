/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final boolean isRobotA = true;

    public final class RobotA {
        // Characteristics
        public static final double kS = 1.;
        public static final double kV = 2.24;
        public static final double kA = 0.252;
        public static final double kP = 0.00843;

        // PCM
        public static final int pcmPort = 20;

        // balling solenoids
        public static final int rMode1 = 0;
        public static final int fMode1 = 1;
        public static final int rMode2 = 3;
        public static final int fMode2 = 2;
        public static final int fContainer = 4;
        public static final int rContainer = 5;
		
		// balling speeds		
    	public static final double superSlowSmallW = 0.4;
    	public static final double superSlowBigW = 0.25;

        // climbing solenoids
        public static final int rSolenoid_lock = 7;
        public static final int fSolenoid_lock = 6;
    }

    public final class RobotB {
        // Characteristics
        public static final double kS = 1.54;
        public static final double kV = 2.15;
        public static final double kA = 1.54;
        public static final double kP = 0.0146;

        // PCMS
        public static final int pcm1Port = 19;
        public static final int pcm2Port = 20;

        // balling solenoids
        public static final int rMode1 = 0;
        public static final int fMode1 = 1;
        public static final int rMode2 = 5;
        public static final int fMode2 = 4;
        public static final int fContainer = 2;
        public static final int rContainer = 3;

		// balling speeds		
    	public static final double superSlowSmallW = 0.2;
    	public static final double superSlowBigW = 0.3;

        // climbing solenoids
        public static final int rSolenoid_lock = 2;
        public static final int fSolenoid_lock = 3;
    }

    public static final int leftJoystick = 1;
    public static final int rightJoystick = 2;
    public static final int mainController = 3;
    public static final int secondaryController = 4;

    public static final int leftFrontMotor = 3;
    public static final int leftBackMotor = 4;
    public static final int rightFrontMotor = 2;
    public static final int rightBackMotor = 1;

    public static final int gyroPort = 11;

    // Balling
    public static final int lWheelsPort = 6;
    public static final int bWheelsPort = 5;
    public static final double fastBigW = 1;
    public static final double slowBigW = 0.3;
    public static final double normalBigW = 0.635;

    public static final double fastSmallW = 1;
    public static final double slowSmallW = 0.4;

    // Climb
    public static final int telescopicMotor = 10;
    public static final int drumMotor = 9;

    public static final double robotTrackWidth = 0.585;

    public static final double maxRotationV = 3.2;
    public static final double maxAngVel = (2 * Math.PI) - 4;
    public static final double maxVel = 3.5;

    public static final double kD = 0;
    public static final int maxCurrent = 40;
    public static final double wheelDiameter = 0.1524;
    public static final double encoderPulsePerRotation = 800.0;
    public static final double pulsePerMeter = encoderPulsePerRotation / (wheelDiameter * Math.PI);

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            robotTrackWidth);

    //public static final double kMaxSpeedMetersPerSecond = 4.559;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final boolean kGyroReversed = false;
}
