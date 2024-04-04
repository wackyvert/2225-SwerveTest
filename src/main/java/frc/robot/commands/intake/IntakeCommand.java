package frc.robot.commands.intake;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;


public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final LightSubsystem lightSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, LightSubsystem lightSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.lightSubsystem = lightSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
       lightSubsystem.turnOnAnimation();
        lightSubsystem.changeAnimation(LightSubsystem.AnimationTypes.solid_orange_strobe);
    }

    @Override
    public void execute() {
        intakeSubsystem.intake();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        intakeSubsystem.stop();
        lightSubsystem.turnOffAnimation();
        lightSubsystem.setTeamColor();
    }
}
