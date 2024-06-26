package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TrapAmpSlowShoot extends Command {
    private final ShooterSubsystem intakeSubsystem;
    private final LightSubsystem lightSubsystem;
    public TrapAmpSlowShoot(ShooterSubsystem intake, LightSubsystem lightSubsystem) {
        this.intakeSubsystem = intake;
        this.lightSubsystem = lightSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intakeSubsystem);
    }

    @Override public void initialize(){
    lightSubsystem.changeAnimation(LightSubsystem.AnimationTypes.solid_green_strobe);
        lightSubsystem.turnOnAnimation();

    }

    @Override
    public void execute(){
        intakeSubsystem.shootSlow();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopShooter();
        lightSubsystem.turnOffAnimation();
        lightSubsystem.setTeamColor();
        //intakeSubsystem.stopTrapMotor();
    }
}
