package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StopDrop extends Command {
     IntakeSubsystem intakeSubsystem;

    public StopDrop(IntakeSubsystem intakeSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(intakeSubsystem);
        this.intakeSubsystem=intakeSubsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.stop();
    }
    

@Override
    public boolean isFinished(){
     return true;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }

    
}
