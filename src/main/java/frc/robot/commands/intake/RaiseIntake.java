package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RaiseIntake extends Command {
     IntakeSubsystem intakeSubsystem;

    public RaiseIntake(IntakeSubsystem intakeSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(intakeSubsystem);
        this.intakeSubsystem=intakeSubsystem;
    }

    @Override
    public void initialize() {
        
    }
    @Override
    public void execute(){
intakeSubsystem.raiseIntake();
    }

@Override
    public boolean isFinished(){
        if(intakeSubsystem.getArmPosition()>=Constants.MotorConstants.UPPER_STOP_POINT)
        {
            return true;
        }else
        {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
    
}
