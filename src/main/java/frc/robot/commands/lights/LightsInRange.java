package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.vision.Vision;

public class LightsInRange extends Command {
    int[] tagIds = new int[]{3, 4, 7, 8};
    Vision vision;
    LightSubsystem lights;
    public LightsInRange(Vision visionInstance, LightSubsystem lightInstance){
        vision=visionInstance;
         lights= lightInstance;
    }
    @Override
    public void execute(){
        if (vision.inRangeOfGivenAprilTag(tagIds)) {
        lights.setGreen();
        }
    }

}
