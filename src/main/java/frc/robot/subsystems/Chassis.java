/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.GroupOfMotors;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class Chassis extends SubsystemBase {
  public GroupOfMotors motorsR, motorsL;
  public PigeonIMU gyro;

  private final DifferentialDriveOdometry odometry;

  private long lastTimeInMiliseconds = 0;
  public final SimpleMotorFeedforward aff;
  private double lastGyroFusedHeading = 0;
  private boolean isReversed = false;
  private double baseAngle;
  private boolean isVelocityMode = true; 

  public Chassis(int leftFront, int leftBack, int rightFront, int rightBack, int gyroPort){
    double kV = Constants.isRobotA ? Constants.RobotA.kV : Constants.RobotB.kV;
    double kS = Constants.isRobotA ? Constants.RobotA.kS : Constants.RobotB.kS;
    double kA = Constants.isRobotA ? Constants.RobotA.kA : Constants.RobotB.kA;
    motorsL = new GroupOfMotors(leftFront, leftBack);
    motorsR = new GroupOfMotors(rightFront, rightBack);
    gyro = new PigeonIMU(gyroPort);
    aff = new SimpleMotorFeedforward(kS, kV, kA);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));
    baseAngle = 0;
    motorsL.invertMotors();
    motorsR.invertMotors();
    motorsR.invertEncoder();
    Shuffleboard.getTab("tab").addString("pose", this::getOdometryString);
    Shuffleboard.getTab("tab").addNumber("gyro", this::getAngle);
  }

  @Override
  public void periodic() {
    lastTimeInMiliseconds = System.currentTimeMillis();
    lastGyroFusedHeading = getAngle();
    odometry.update(Rotation2d.fromDegrees(getAngle()), getLeftDistance(),
    getRightDistance());
  }
  
  public String getOdometryString(){ // returns the position of the robot
    return odometry.getPoseMeters().toString();
  }

  public double getTurnRate(){ // returns the turn rate of the robot
    if(lastTimeInMiliseconds == 0) return 0;
    return (getAngle() - lastGyroFusedHeading) / (System.currentTimeMillis() - lastTimeInMiliseconds);

  }

  public void resetOdometry(){ // resets the position, encoder and gyro of the robot
    resetEncoders();
    resetGyro();
    baseAngle = 0;
    odometry.resetPosition(new Pose2d(0., 0., new Rotation2d(0)), new Rotation2d(0));
  }

  public void resetOdometry(final Pose2d pose) { // sets your pose to the desired pose
    resetEncoders();
    resetGyro(pose.getRotation().getDegrees());
    odometry.resetPosition(pose, pose.getRotation());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() { // returns the wheel speeds of the robot
    return new DifferentialDriveWheelSpeeds(getLeftVel(), getRightVel());
  }

  public Pose2d getPose() { // returns the position of the robot
    return odometry.getPoseMeters();
  }

  public double getLeftVel(){ // returns the velocity of the left motors of the robot in m/s
    if(isReversed){
      return -motorsR.getVelocity();
    } else {
      return motorsL.getVelocity();
    }
  }

  public double getRightVel(){ // returns the velocity of the right motors of the robot in m/s
    if(isReversed){
      return -motorsL.getVelocity();
    } else {
      return motorsR.getVelocity();
    }
  }

  public boolean isVelocityMode(){
    return isVelocityMode;
  }
  
  public void setVelocityMode(boolean isVelMode){
    isVelocityMode = isVelMode;
  }

  public void setPower(double left, double right){ // sets the power of the motors
    if (isReversed){
      left = -right;
      right = -left;
    }
    motorsL.setPower(left);
    motorsR.setPower(right);
  }

  public void setVelocity(double left, double right){ // sets the velocity of the motors in m/s
    if (isReversed){
      left = -right;
      right = -left;
    }
    System.out.println("setting left");
    motorsL.setVelocity(left, getAff());
    System.out.println("setting right");
    motorsR.setVelocity(right, getAff());
  }

  public void setVelocityAndAngular(double vel, double angularVel){ // calculates the velocity to the sides of the chassis and sets it
    ChassisSpeeds speeds = new ChassisSpeeds(vel, 0, angularVel);
    DifferentialDriveWheelSpeeds wspd =  Constants.kDriveKinematics.toWheelSpeeds(speeds); 

    System.out.println("speeds=" + speeds);
    System.out.println("wsp=" + wspd);
    setVelocity2(wspd.leftMetersPerSecond,
     wspd.rightMetersPerSecond);
  }

  public void setVelocityWithAcceleration(double left, double right){
    if (isReversed){
      left = -right;
      right = -left;
    }
      motorsL.setVelocityWithAcceleration(left, getAff());
      motorsR.setVelocityWithAcceleration(right, getAff());
  }

  public void setRelVelocity(double left, double right){ // sets the velocity between maxSpeed to -maxSpeed
    left *= Constants.maxVel;
    right *= Constants.maxVel;
      setVelocity2(left, right);
  }

  public SimpleMotorFeedforward getAff(){ // returns the arbitrary feed forward of the robot
    return aff;
  }

  public double getAngle(){ // returns the raw angle of the robot
    if(isReversed){
      return (gyro.getFusedHeading() + baseAngle) * (Constants.kGyroReversed ? -1.0 : 1.0) + 180;
    } else {
      return (gyro.getFusedHeading() + baseAngle) * (Constants.kGyroReversed ? -1.0 : 1.0);
    }
  }

  public double getAngle2Pose(Pose2d pose){
    Translation2d translation2d = pose.getTranslation().minus(getPose().getTranslation());
    return new Rotation2d(translation2d.getX(), translation2d.getY()).getDegrees();
  }

  public double getDistance2Pose(Pose2d pose){
    return pose.getTranslation().getDistance(getPose().getTranslation());
  }

  public double normalizeAngle(double angle){ // returns the angle between 180 to -180
	  return Math.IEEEremainder(angle, 360);
  }

  public double getNormalizedAngle(){ // returns the angle of the robot between 180 to -180
    return normalizeAngle(getAngle());
  }

  public double diffAngle(double reqAngle, double curAngle){ // returns the shortest angle to what you want
    double a1 = normalizeAngle(reqAngle) - normalizeAngle(curAngle);
    if (a1 <= -180) {
      return a1 + 360;
    } else if(a1 > 180) {
      return a1 - 360;
    } else {
      return a1;
    }
  }

  public double getLeftDistance(){ // returns the distance passed since reset
    if(isReversed){
      return -motorsR.getDistance();
    } else {
      return motorsL.getDistance();
    }
  }

  public void resetGyro(){ // resets the gyro to 0 angle
    gyro.setFusedHeading(0);
    baseAngle = 0;
  }

  public void resetGyro(double angle){
    baseAngle = 0;
    baseAngle = angle - getAngle();
  }

  public void resetEncoders(){ // resets the encoders
    motorsL.resetEncoder();
    motorsR.resetEncoder();
  }

  public double getRightDistance(){ // returns the distance passed on the right side of the chassis
    if(isReversed){
      return -motorsL.getDistance();
    } else {
      return motorsR.getDistance();
    }
  }

  public double getDistance(){ // returns the distance of the chassis itself
    return (getLeftDistance() + getRightDistance())/2;
  }

  public void reverseRobot(boolean isReversed){ // reverses everything in the robot
    if(isReversed != this.isReversed) {
      this.isReversed = isReversed;
      Pose2d pose = odometry.getPoseMeters();
      odometry.resetPosition(pose, pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }

  }

  public CommandBase getReverseRobotCommand(boolean reverse){
    return new InstantCommand(() ->{
      reverseRobot(reverse);
    }, this);
  }

  public boolean inVisionMode(XboxController controller) { // returns true when Y is pressed
    return controller != null && controller.getYButton() && SmartDashboard.getNumber("VisionDistance", 0) > 0;
  }
  public double getDistanceDiff(){
    return getLeftDistance() - getRightDistance();
  }

  public void driveToBall(double speed) { // sets the needed power to the motors to go to the selected ball
    double distance = SmartDashboard.getNumber("VisionDistance", 0);
    double angle = SmartDashboard.getNumber("VisionAngle", 0);
    double radius = distance / (2 * Math.sin(angle * Math.PI / 180));
    double k = Constants.robotTrackWidth * 100 / 2;
    double left = speed * (1 + (k / radius));
    double right = speed * (1 - (k / radius));
    if(speed > 0)
      System.out.printf(" Left = %.2f , right = %.2f , angle = %3.1f\n", left, right, angle);
    setRelVelocity(left, right);
  }

  public CommandBase driveToBallCommand(double speed) {
    /*return new RunCommand(() ->{
      driveToBall(speed);
    }).raceWith(new WaitUntilCommand(() ->{
      return true;
    })).;*/
    return new FunctionalCommand(() -> {},
    () -> {
      driveToBall(speed);
    }, (interrupted) -> {
      setRelVelocity(0, 0);
    }, () -> {return SmartDashboard.getNumber("VisionDistance", 0) == 0;});
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Angle", this::getAngle, null);
    builder.addDoubleProperty("Left Dist", this::getLeftDistance, null);
    builder.addDoubleProperty("Right Dist", this::getRightDistance, null);
    builder.addDoubleProperty("Distance Diff", this::getDistanceDiff, null);
    builder.addDoubleProperty("Distance", this::getDistance, null);
    builder.addBooleanProperty("in velocity mode?", this::isVelocityMode, null);
  }

  public static final double K1 = 3;
  public static final double K2 = 2;
  public void setVelocity2(double left, double right) {
    // if(left * right <= 0) {
    //   setVelocity(left, right);
    //   return;
    // }
    if (isReversed){
      left = -right;
      right = -left;
    }
    double diff = (left - right);
    double l = getAff().calculate(left, motorsL.getAccelForSpeed(left)) + K1 * diff;
    double r = getAff().calculate(right,  motorsR.getAccelForSpeed(right)) - K1 * diff;
    double sgn = Math.signum(left);
    if (diff * sgn > 0) {
      // Outer is left - power is Frictionn + CEMF + angular
      r =  r - sgn * (1-right/left)*K2;
    } else {
      l = l - sgn * (1-left/right)*K2;
    }
    //System.out.println("power: " + l/12 + " / " + r/12);
    motorsL.setVelocity(left, l/12);
    motorsR.setVelocity(right, r/12);
  }
}
