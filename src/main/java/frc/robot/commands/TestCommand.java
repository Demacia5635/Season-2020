package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class TestCommand extends CommandBase{

    private Chassis chassis;
    private double velocity;
    private double sumOfErrors = 0;
    private double lastError = 0;
    private boolean fixAngle;
    private static final double kP = 1/90;
    private static final double kI = kP/10;
    private static final double kD = 0;

    public TestCommand(Chassis subsystem, double velocity, boolean fixAngle){
        this.chassis = subsystem;
        this.velocity = velocity;
        this.fixAngle = fixAngle;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.motorsL.resetEncoder();
		chassis.motorsR.resetEncoder();
        chassis.resetGyro();
    }

    @Override
    public void execute() {
        if (fixAngle){
            double error = chassis.getAngle();
            double diffOfErrors = error - lastError;
            sumOfErrors += error;
            double fix = kP*error + kI*sumOfErrors + kD*diffOfErrors;
            chassis.setVelocity(velocity - fix, velocity + fix);
            error = lastError;
        } else {
            chassis.setVelocity(velocity, velocity);
        }
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setVelocity(0, 0);
    }
}