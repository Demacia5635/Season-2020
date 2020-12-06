/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.RamseteCommandDemacia;
import frc.robot.commands.TelescopeCommand;
import frc.robot.commands.TurnByDegrees;
import frc.robot.commands.VelocityDrive;
import frc.robot.commands.turnWhileMoving;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.utils.InputDev;
import frc.robot.utils.TrajectoryUtility;
import frc.robot.Constants.RobotA;
import frc.robot.Constants.RobotB;
import frc.robot.commands.AngleTest;
import frc.robot.commands.BallsManager;
import frc.robot.commands.DefenseCommand;
import frc.robot.commands.DrumCommand;
import frc.robot.commands.Go2Pose;
import frc.robot.commands.GoStraight;
import frc.robot.commands.PowerDrive;
import frc.robot.subsystems.Balling;
import frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.Compressor;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chassis chassis = new Chassis(Constants.leftFrontMotor, Constants.leftBackMotor,
      Constants.rightFrontMotor, Constants.rightBackMotor, Constants.gyroPort);
  // private final Roulette m_roulette = new Roulette();
  public final Climb m_climb = new Climb();
  private final Balling m_balling = new Balling();

  public enum DrivingMethods {
    JS_1, JS_2, XBOX_2Sticks, XBOX_1Stick, XBOX_Orbit, Einziger, YorgenMode;
  }

  private Compressor compressor;
  private final XboxController mainController = new XboxController(Constants.mainController);
  private final XboxController secondaryController = new XboxController(Constants.secondaryController);
  SendableChooser<CommandBase> autoChooser = new SendableChooser<>();
  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  // SendableChooser<DrivingMethods> method_chooser = new SendableChooser<>();

  private final BallsManager ballingCommand = new BallsManager(m_balling, mainController, secondaryController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (Constants.isRobotA) {
      compressor = new Compressor(RobotA.pcmPort);
    } else {
      compressor = new Compressor(RobotB.pcm1Port);
    }

    chassis.setDefaultCommand(new VelocityDrive(chassis, mainController));
    m_climb.drumSubsystem.setDefaultCommand(new DrumCommand(m_climb, secondaryController));
    m_climb.telescopeSubsystem.setDefaultCommand(new TelescopeCommand(m_climb, secondaryController));

    // Configure the button bindings
    configureButtonBindings();
    autoChooser.setDefaultOption("[t4] normal auto (x: 2.68, y: -2.65, angle: 180)", getAutoTest4());
    autoChooser.addOption("t5] (x: 2.68, y: -4.65, angle: 180)", getAutoTest5());
    autoChooser.addOption("cross auto line", new GoStraight(1, 1.5, chassis, true));
    autoChooser.addOption("t6] (x: 1, y: -2.65", getAutoTest6());
    autoChooser.addOption("t7] (x:1, y:-3.47, angle:180)", getAutoTest7());
    autoChooser.addOption("t8]", getAutoTest8());
    autoChooser.addOption("t9] (rendevew autonomus)", getAutoTest9());
    autoChooser.addOption("t10] (puts balls and go left)", getAutoTest10());
    autoChooser.addOption("t[11]", getAutoTest11());
    SmartDashboard.putData("auto", autoChooser);
    /*
     * m_chooser.setDefaultOption("Power", new PowerDrive(chassis, inputDevice));
     * m_chooser.addOption("Velocity", new VelocityDrive(chassis, inputDevice));
     * SmartDashboard.putData("Driving Type", m_chooser);
     */
    SmartDashboard.putData("reset gyro and odometry", new InstantCommand(() ->{
      chassis.resetOdometry();
    }));
    SmartDashboard.putData("test throw 3 balls", m_balling.getThrow3BallsCommand()
    .andThen(new WaitCommand(1)
    ,m_balling.getUnContainCommand()));
    // SmartDashboard.putData("go2Pose", new InstantCommand(() ->{
    //   chassis.resetOdometry(new Pose2d(3 , -2, Rotation2d.fromDegrees(180)));
    // }, chassis).andThen(new Go2Pose(1, new Translation2d(2, -2), chassis, true)));
    // SmartDashboard.putData("test reset gyro" , new InstantCommand(() ->{
    //   chassis.resetGyro(90);
    // }, chassis));
    // SmartDashboard.putData("go straight", getMovingStraightCommand());
    // SmartDashboard.putData("turn by degrees", getTurnByDegreesCommand());
    // SmartDashboard.putData("angle test", new AngleTest(chassis));
    // SmartDashboard.setDefaultString("Path Following", "Test");
    // SmartDashboard.putData("Path Follow test", getPathFollowingCommand());

    /*
     * method_chooser.setDefaultOption("2 Joystick", DrivingMethods.JS_2);
     * method_chooser.addOption("1 Joystick", DrivingMethods.JS_1);
     * method_chooser.addOption("xbox 1 stick", DrivingMethods.XBOX_1Stick);
     * method_chooser.addOption("xbox 2 sticks", DrivingMethods.XBOX_2Sticks);
     * method_chooser.addOption("XBOX - ORBIT - ULTRA - MODE",
     * DrivingMethods.XBOX_Orbit); method_chooser.addOption("Yorgen Mode",
     * DrivingMethods.YorgenMode); method_chooser.addOption("Einziger",
     * DrivingMethods.Einziger); SmartDashboard.putData("Driving method",
     * method_chooser);
     */

    SmartDashboard.setDefaultNumber("Velocity", 1);
    SmartDashboard.setDefaultNumber("Distance", 1);
    SmartDashboard.setDefaultNumber("Degrees", 90);
    SmartDashboard.setDefaultBoolean("Accuracy", false);
  }

  /*
   * public void setDrivinigMethod(DrivingMethods method) { switch (method) { case
   * JS_2: inputDevice = new InputDev(left, right); break; case JS_1: inputDevice
   * = new InputDev(left); break; case XBOX_2Sticks: inputDevice = new
   * InputDev(mainController, false, false); break; case XBOX_1Stick: inputDevice
   * = new InputDev(mainController, true, false); break; case XBOX_Orbit:
   * inputDevice = new InputDev(mainController, true, true); break; case
   * YorgenMode: inputDevice = new InputDev(mainController,
   * DrivingMethods.YorgenMode); break; case Einziger: inputDevice = new
   * InputDev(mainController, DrivingMethods.Einziger); break; } }
   * 
   * public InputDev getInputDevice() { return inputDevice; }
   * 
   * public DrivingMethods getDrivingMethod() { return
   * inputDevice.getDrivingMethod(); }
   */

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // general
    final JoystickButton startButtonMain = new JoystickButton(mainController, 8);
    final JoystickButton backButtonSecondary = new JoystickButton(secondaryController, 7); // abort mission
    final JoystickButton startButtonSecondary = new JoystickButton(secondaryController, 8);

    startButtonSecondary.whenHeld(new InstantCommand(() -> {
      m_climb.setInClimbing(!m_climb.getInClimbing());
      if(!m_climb.getInClimbing()){
        ballingCommand.schedule();
      }
      else{
        ballingCommand.cancel();
      }
    }));

    startButtonMain.whenHeld(new InstantCommand(() -> {
      if(chassis.isVelocityMode()){
        new PowerDrive(chassis, new InputDev(mainController, DrivingMethods.YorgenMode)).schedule();
        chassis.setVelocityMode(false);
      }
      else{
        new VelocityDrive(chassis, mainController).schedule();
        chassis.setVelocityMode(true);
      }
    }));

     backButtonSecondary.whenPressed(new InstantCommand(() -> {
       if (compressor.getClosedLoopControl()) {
         compressor.stop();
       } else {
         compressor.start();
       }
     }));

     startButtonMain.whenPressed(new InstantCommand(() -> {
       if (compressor.getClosedLoopControl()) {
         compressor.stop();
       } else {
         compressor.start();
       }
     }));
  }

  public CommandBase getRamseteCommand() {
    chassis.resetOdometry();
    double kS = Constants.isRobotA ? Constants.RobotA.kS : Constants.RobotB.kS;
    double kV = Constants.isRobotA ? Constants.RobotA.kV : Constants.RobotB.kV;
    double kA = Constants.isRobotA ? Constants.RobotA.kA : Constants.RobotB.kA;
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(kS, kV, kA), Constants.kDriveKinematics, 10);

    // Create config for trajectory
    final TrajectoryConfig config = new TrajectoryConfig(1.5, // Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared / 10)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    final Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(),

        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0, 1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 1, new Rotation2d(Units.degreesToRadians(90))), config);

    final RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, chassis::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics,
        // RamseteCommand passes volts to the callback
        chassis::setVelocity, chassis);

    return ramseteCommand;
  }

  public RamseteCommand getPathFollowingCommand() {
    final String name = SmartDashboard.getString("Path Following", "Test");
    final Trajectory trj = TrajectoryUtility.getTrajectory(name);
    if (trj == null) {
      return null;
    }
    final Pose2d startPos = trj.sample(0).poseMeters;
    chassis.resetOdometry(startPos);
    final RamseteCommand cmd = new RamseteCommandDemacia(trj, chassis::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics,
        chassis::setVelocity, chassis);

    return cmd;
  }

  public CommandBase getMovingStraightCommand() {
    return new GoStraight(1, 1 , 10, chassis, true);
  }

  public TurnByDegrees getTurnByDegreesCommand() {
    return new TurnByDegrees(SmartDashboard.getNumber("Degrees", 90), chassis,
        SmartDashboard.getBoolean("with pid", false), false);
  }

  public CommandBase getAutoTest1(){ //first test in whatsapp
    return new InstantCommand(() -> {
      //chassis.resetOdometry(new Pose2d(3.1, -0.8, Rotation2d.fromDegrees(-135)));
    }).andThen(new PrintCommand("cur angle: " + chassis.getAngle()),  m_balling.getMiddleCommand(), m_balling.getShootingCommand()
    ,new WaitCommand(2), m_balling.getCollectingCommand()
    ,new TurnByDegrees(0, chassis, false, true)
    ,new GoStraight(1, 4.2, 0, chassis, false) //distance should be 5 meters
    , new TurnByDegrees(-160, chassis, false, true)
    , m_balling.getShootingCommand()
    );
  } 

  public CommandBase getAutoTest2(){
    return new InstantCommand(() -> {
      chassis.resetOdometry(new Pose2d(3.3, -7.5, Rotation2d.fromDegrees(155)));
    }, chassis).andThen(m_balling.getLowCommand() 
    ,m_balling.getShootingCommand()
    ,m_balling.getCollectingCommand()
    ,new TurnByDegrees(0, chassis, false, true)
    ,new GoStraight(1.5, 2.9, 0, chassis, true) //should be 2.5 meter distance
    ,new TurnByDegrees(170, chassis, false, true)
    ,m_balling.getShootingCommand()
    ,new GoStraight(2, 2, chassis, true) //should be 2 meters distance
    );
  }

  double startDistance;
  public CommandBase getAutoTest3(){
    return new InstantCommand(() ->{
      chassis.resetOdometry(new Pose2d(3.45,5.75 , Rotation2d.fromDegrees(175)));
    }).andThen(m_balling.getLowCommand()
    ,new WaitCommand(0.5)
    ,m_balling.getShootingCommand()
    ,new TurnByDegrees(6, chassis, false, true)
    ,new GoStraight(1.5, 5.75, 6, chassis, false) //stop at end may be true
      .alongWith(new WaitUntilCommand(() ->{
        return chassis.getDistance() - startDistance > 4;
      }).beforeStarting(() -> { startDistance = chassis.getDistance();})
      .andThen(m_balling.getCollecShootingCommand()))
      ,new TurnByDegrees(67, chassis, false, true)
      ,m_balling.getCollectingCommand()
      ,new GoStraight(1, 0.7, 67, chassis, true)
      ,m_balling.getMiddleCommand()
      ,new TurnByDegrees(169, chassis, false, true)
      ,m_balling.getShootingCommand()
    );
  }

  public CommandBase getAutoTest4(){ 
    //puts balls in low, go get more balls 
    //from our trench and puts them in the low port
    //starts in front of the target, back bumpers touches the init line
    return new InstantCommand(() ->{
      chassis.resetOdometry(new Pose2d(2.61, -2.4, Rotation2d.fromDegrees(180)));
    }, chassis).andThen(
      m_balling.getMiddleCommand()      
      ,m_balling.getContainCommand()
      ,m_balling.getThrow3BallsCommand()
      ,new GoStraight(2, 0.35, 180, chassis, false)
      ,new Go2Pose(0.6, new Translation2d(0.6, -2.4), chassis, true)
      ,m_balling.getUnContainCommand()
      ,new WaitCommand(1)
      ,m_balling.getStopMotorsCommand()
      ,new GoStraight(-1, -1, 180,  chassis, true)
      ,new TurnByDegrees(new Translation2d(6.4, -0.7), 2.5, chassis)
      ,m_balling.getLowCommand()
      ,m_balling.getCollectingCommand()
      ,new Go2Pose(2.4, new Translation2d(4.3, -0.9), chassis, false)
      ,new Go2Pose(0.6, new Translation2d(6.4, -0.7), chassis, true)
      ,new TurnByDegrees(new Translation2d(2.7, -2.4), 2.5, chassis)
      ,m_balling.getStopMotorsCommand()
      ,m_balling.getMiddleCommand()
      ,new Go2Pose(2.5, new Translation2d(2.1, -2.3), chassis, false)
      ,m_balling.getSlowShootingCommand()
      ,new Go2Pose(1.2, new Translation2d(1.9, -2.4),  chassis, true)
      ,new Go2Pose(0.6, new Translation2d(0.72, -2.4),  chassis, true)
      ,m_balling.getUnContainCommand()
      ,new WaitCommand(1.5)
      ,m_balling.getStopMotorsCommand()
      );

  }

  public CommandBase getAutoTest5(){ 
    //puts balls in low, go get more balls 
    //from our trench and puts them in the low port
    //starts 2 meters from target, back bumpers touches the init line
    return new InstantCommand(() ->{
      chassis.resetOdometry(new Pose2d(2.61, -4.4, Rotation2d.fromDegrees(180)));
    }, chassis).andThen(
      m_balling.getMiddleCommand()      
      ,m_balling.getContainCommand()
      ,m_balling.getThrow3BallsCommand()
      ,new Go2Pose(2, new Translation2d(2., -2.5), chassis, false)
      ,new Go2Pose(0.6, new Translation2d(0.6, -2.4), chassis, true)
      ,m_balling.getUnContainCommand()
      ,new WaitCommand(1)
      ,m_balling.getStopMotorsCommand()
      ,new GoStraight(-1, -1, 180,  chassis, true)
      ,new TurnByDegrees(new Translation2d(6.3, -1), 2.5, chassis)
      ,m_balling.getLowCommand()
      ,m_balling.getCollectingCommand()
      ,new Go2Pose(2.4, new Translation2d(4.3, -0.9), chassis, false)
      ,new Go2Pose(0.6, new Translation2d(6.4, -0.7), chassis, true)
      ,new TurnByDegrees(160, chassis,false,false)
      ,m_balling.getStopMotorsCommand()
      ,m_balling.getMiddleCommand()
      ,new Go2Pose(2.5, new Translation2d(2.1, -2.3), chassis, false)
      ,m_balling.getSlowShootingCommand()
      ,new Go2Pose(1.2, new Translation2d(1.9, -2.4),  chassis, true)
      ,new Go2Pose(0.6, new Translation2d(0.6, -2.4),  chassis, true)
      ,m_balling.getUnContainCommand()
      ,new WaitCommand(1.5)
      ,m_balling.getStopMotorsCommand()
      );
  }

  public CommandBase getAutoTest6(){ 
    //throw balls, go get more balls 
    //from our trench and puts them in the low port
    //starts in front of our trench, front bumper touches the init line
    return new InstantCommand(() ->{
      chassis.resetOdometry(new Pose2d(3.49, -0.7, Rotation2d.fromDegrees(180)));
    }, chassis).andThen(
      m_balling.getMiddleCommand()      
      ,m_balling.getContainCommand()
      ,m_balling.getThrow3BallsCommand()
      ,new WaitCommand(0.5)
      ,m_balling.getUnContainCommand()
      ,new WaitCommand(1)
      ,m_balling.getStopMotorsCommand()
      ,new TurnByDegrees(new Translation2d(6.4, -1.), 2.5, chassis)
      ,m_balling.getCollectingCommand()
      ,new Go2Pose(2.4, new Translation2d(4.3, -0.9), chassis, false)
      ,new Go2Pose(0.6, new Translation2d(6.4, -0.7), chassis, true)
      ,new TurnByDegrees(new Translation2d(2.7, -2.4), 2.5, chassis)
      ,m_balling.getStopMotorsCommand()
      ,m_balling.getMiddleCommand()
      ,new Go2Pose(2.5, new Translation2d(2.1, -2.3), chassis, false)
      ,m_balling.getSlowShootingCommand()
      ,new Go2Pose(1.2, new Translation2d(1.9, -2.4),  chassis, true)
      ,new Go2Pose(0.6, new Translation2d(0.6, -2.4),  chassis, true)
      ,m_balling.getUnContainCommand()
      ,new WaitCommand(1.5)
      ,m_balling.getStopMotorsCommand()
      );
  }

  public CommandBase getAutoTest7(){ 
    //puts balls in the low port, go get more balls 
    //from our trench and puts them in the low port
    //starts in ftont of our trench, front bumpers thouches the init line
    return new InstantCommand(() ->{
      chassis.resetOdometry(new Pose2d(3.5, -0.7, Rotation2d.fromDegrees(180)));
    }, chassis).andThen(
      m_balling.getLowCommand()      
      ,m_balling.getContainCommand()
      ,m_balling.getThrow3BallsCommand()
      ,new Go2Pose(2, new Translation2d(2.1, -2.4), chassis, false)
      ,new Go2Pose(0.6, new Translation2d(0.6, -2.4), chassis, true)
      ,m_balling.getUnContainCommand()
      ,new WaitCommand(1)
      ,m_balling.getStopMotorsCommand()
      ,new GoStraight(-1, -1, 180,  chassis, true)
      ,new TurnByDegrees(new Translation2d(4.3, -0.9), 2.5, chassis)
      ,m_balling.getLowCommand()
      ,m_balling.getCollectingCommand()
      ,new Go2Pose(2.4, new Translation2d(4.3, -0.9), chassis, false)
      ,new Go2Pose(0.6, new Translation2d(6.4, -0.7), chassis, true)
      ,new TurnByDegrees(new Translation2d(2.7, -2.4), 2.5, chassis)
      ,m_balling.getStopMotorsCommand()
      ,m_balling.getMiddleCommand()
      ,new Go2Pose(2.5, new Translation2d(2.1, -2.3), chassis, false)
      ,m_balling.getSlowShootingCommand()
      ,new Go2Pose(1.2, new Translation2d(1.9, -2.4),  chassis, true)
      ,new Go2Pose(0.6, new Translation2d(0.6, -2.4),  chassis, true)
      ,m_balling.getUnContainCommand()
      ,new WaitCommand(1.5)
      ,m_balling.getStopMotorsCommand()
      );
  }

  public CommandBase getBaseAutoTest8(){ 
    //puts balls in low
    //starts in front of the target, back bumpers touches the init line
    return new InstantCommand(() ->{
      chassis.resetOdometry(new Pose2d(2.75, -2.4, Rotation2d.fromDegrees(180)));
    }, chassis).andThen(
      m_balling.getMiddleCommand()      
      ,m_balling.getContainCommand()
      ,m_balling.getThrow3BallsCommand()
      ,new GoStraight(2, 0.35, 180, chassis, false)
      ,new Go2Pose(0.6, new Translation2d(0.6, -2.4), chassis, true)
      ,m_balling.getUnContainCommand()
      );
    }

    public CommandBase getAutoTest8(){
      return getBaseAutoTest8().andThen(
        new WaitCommand(2.5)
        ,m_balling.getStopMotorsCommand()
      );
    }

    public CommandBase getAutoTest9(){
      //throw balls, go the counter the enemy rendevew
      //starts with the right bumper  2 meters from edge trench,
      //the front bumper on the init line
      return new InstantCommand(() ->{
        chassis.resetOdometry(new Pose2d(3.49, -5.8, Rotation2d.fromDegrees(180)));
      }, chassis).andThen(
        m_balling.getLowCommand()
        ,m_balling.getShootingCommand()
        ,new GoStraight(-1.5, -2, chassis, false)
        ,m_balling.getUnContainCommand()
        ,new TurnByDegrees(new Translation2d(10.1, -5.1), 2.7, chassis)
        ,m_balling.getCloseCommand()
        ,m_balling.getStopMotorsCommand()
        ,new Go2Pose(2.2, new Translation2d(10.1, -5.1), chassis, false)
        ,new Go2Pose(2.2, new Translation2d(10.5, -3), chassis, true) 
      );
    }

    public CommandBase getAutoTest10(){
    //puts balls in low and go left
    //starts in front of the target, back bumpers touches the init line
      return getBaseAutoTest8().andThen(
        new WaitCommand(1)
        ,m_balling.getStopMotorsCommand()
        ,new GoStraight(-1.5, -1, chassis, false)
        ,new TurnByDegrees(new Translation2d(1.6, -4), 2.7, chassis)
        ,new Go2Pose(1.5, new Translation2d(1.6, -4), chassis, true)
      );
    }

    public CommandBase getAutoTest11(){
      //throw balls, go the counter the enemy rendevew, takses them and throw them to our sector
      //starts with the right bumper  2 meters from edge trench,
      //the front bumper on the init line
      return new InstantCommand(() ->{
        chassis.resetOdometry(new Pose2d(3.49, -5.8, Rotation2d.fromDegrees(180)));
      }, chassis).andThen(
        m_balling.getLowCommand()
        ,m_balling.getShootingCommand()
        ,new GoStraight(-1.5, -2, chassis, false)
        ,m_balling.getUnContainCommand()
        ,new TurnByDegrees(new Translation2d(8.2, -5.3), 2.7, chassis)
        ,m_balling.getCloseCommand()
        ,m_balling.getStopMotorsCommand()
        ,new Go2Pose(2.2, new Translation2d(8.2, -5.3), chassis, false)
        ,m_balling.getLowCommand()
        ,m_balling.getCollectingCommand()
        ,new WaitCommand(1.5)
        ,new Go2Pose(2.2, new Translation2d(9.8, -5), chassis, false)
        ,m_balling.getShootingCommand()
        ,new TurnByDegrees(160, chassis, false, true)
        ,m_balling.getCollectingCommand()
        ,new TurnByDegrees(new Translation2d(10.5, -3), 2.5, chassis)
        ,new Go2Pose(2.2, new Translation2d(10.5, -3), chassis, true)
        ,m_balling.getShootingCommand()
        ,new TurnByDegrees(-160, chassis, false, true)        
        ,m_balling.getUnContainCommand()  
      );
    }

    public CommandBase getAutoTest12(){
      //throw balls, go the counter the enemy rendevew, takses them and throw them to our sector
      //starts with the right bumper  2 meters from edge trench,
      //the front bumper on the init line
      return new InstantCommand(() ->{
        chassis.resetOdometry(new Pose2d(3.49, -5.8, Rotation2d.fromDegrees(180)));
      }, chassis).andThen(
        m_balling.getLowCommand()
        ,m_balling.getShootingCommand()
        ,new GoStraight(-1.5, -2, chassis, false)
        ,m_balling.getUnContainCommand()
        ,new TurnByDegrees(new Translation2d(8.2, -5.3), 2.7, chassis)
        ,m_balling.getCloseCommand()
        ,m_balling.getStopMotorsCommand()
        ,new Go2Pose(2.2, new Translation2d(8.2, -5.3), chassis, false)
        ,m_balling.getLowCommand()
        ,m_balling.getCollectingCommand()
        ,new WaitCommand(1.5)
        ,new Go2Pose(2.2, new Translation2d(9.7, -4.7), chassis, false)
        ,new TurnByDegrees(-90, chassis, false, false)
        ,m_balling.getShootingCommand()
        ,new TurnByDegrees(160, chassis, false, true)
        ,m_balling.getCollectingCommand()
        ,new Go2Pose(2.2, new Translation2d(10.5, -3), chassis, true)
        ,m_balling.getShootingCommand()
        ,new TurnByDegrees(-160, chassis, false, true)        
        ,m_balling.getUnContainCommand()  
      );
    }
    
    public CommandBase autoDefense(){
      return new InstantCommand(() ->{
        chassis.resetOdometry(new Pose2d(3.85, -5.824, Rotation2d.fromDegrees(0)));
        chassis.reverseRobot(true);
      }, chassis).andThen(
        new Go2Pose(3., new Translation2d(11.9, -5.824), chassis, false)
        ,new TurnByDegrees(new Translation2d(11.9, -5.), 2.5, chassis)
        ,new DefenseCommand(new Translation2d(11.9, -4.2)
        ,  new Translation2d(11.9, -7.2), chassis)
      );
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    /*
     * return new InstantCommand(() -> { chassis.setVelocity(0.5, 0.5);
     * }).withTimeout(2).andThen(new InstantCommand(() -> { chassis.setVelocity(0,
     * 0); }));
     */
    // m_climb.setInClimbing(false);
    // return new turnWhileMoving(90, 1, chassis);
    return autoChooser.getSelected();
  }

  public void startTeleop() {
    m_climb.setInClimbing(false);
    ballingCommand.schedule();

    /*
     * setDrivinigMethod(method_chooser.getSelected()); if (teleopCommand instanceof
     * PowerDrive) { ((PowerDrive) teleopCommand).setInputDevice(inputDevice); }
     * else { ((VelocityDrive) teleopCommand).setInputDevice(inputDevice); } return
     * teleopCommand;
     */
  }

  public void enableInit() {
    //chassis.resetOdometry();
  }
}
