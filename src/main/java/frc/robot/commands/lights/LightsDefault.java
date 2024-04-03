package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LightSubsystem.AnimationTypes;

public class LightsDefault extends Command{
    LightSubsystem lights;
    public LightsDefault(LightSubsystem lightsInstance){
        lights=lightsInstance;
        addRequirements(lights);
    }

    @Override
    public void initialize(){
        lights.changeAnimation(AnimationTypes.Rainbow);
    }
}
