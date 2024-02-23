package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimbCommand extends Command {
    ClimberSubsystem climberSubsystem;

    public ClimbCommand(ClimberSubsystem climberSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(climberSubsystem);
        this.climberSubsystem=climberSubsystem;
    }

    @Override
    public void initialize() {
        climberSubsystem.climb();
    }



    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }
}
