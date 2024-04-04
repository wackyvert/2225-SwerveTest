package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class StartShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final LightSubsystem lightSubsystem;

    public StartShootCommand(ShooterSubsystem shooterSubsystem, LightSubsystem lightSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.lightSubsystem=lightSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        lightSubsystem.changeAnimation(LightSubsystem.AnimationTypes.solid_purple_strobe);
        lightSubsystem.turnOnAnimation();
        shooterSubsystem.shoot();

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {


    }
}
