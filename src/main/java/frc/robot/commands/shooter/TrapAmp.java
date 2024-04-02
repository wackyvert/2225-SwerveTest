package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;



public class TrapAmp extends Command {
    private final ShooterSubsystem intakeSubsystem;
    public TrapAmp(ShooterSubsystem intake) {
        this.intakeSubsystem = intake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakeSubsystem.runTrapMotor();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //intakeSubsystem.stopShooter();
        intakeSubsystem.stopTrapMotor();
    }
}
