// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.climber.ClimbCommand;
import frc.robot.commands.climber.DescendCommand;
import frc.robot.commands.intake.DropIntake;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OutttakeCommand;
import frc.robot.commands.intake.RaiseIntake;
import frc.robot.commands.intake.RunIntakeSeq;
import frc.robot.commands.intake.StopDrop;
import frc.robot.commands.intake.StopIntakeSeq;
import frc.robot.commands.lights.LightsDefault;
import frc.robot.commands.lights.LightsInRange;
import frc.robot.commands.shooter.*;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{


  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                  "swerve"));
 // Vision vision = new Vision(drivebase);
  LightSubsystem lights = new LightSubsystem();
  ClimberSubsystem climber = new ClimberSubsystem();
  ShooterSubsystem shooter  = new ShooterSubsystem();
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);
  IntakeSubsystem intake = new IntakeSubsystem();
  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  XboxController driverXbox2 = new XboxController(1);
  Command driveFieldOrientedDirectAngle;
  Command driveFieldOrientedDirectAngleSim;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * @throws IOException
   */
  public RobotContainer() throws IOException
  {// Configure the trigger bindings
    configureBindings();

//    PathPlanner Config
    NamedCommands.registerCommand("RunIntake", new SequentialCommandGroup(new RunIntakeSeq(intake), new WaitCommand(2.0), new StopIntakeSeq(intake)));
    NamedCommands.registerCommand("ShootCommand", new ParallelCommandGroup(new ShootCommand(shooter, lights), new OutttakeCommand(intake)));
    NamedCommands.registerCommand("DropIntake", new SequentialCommandGroup(new DropIntake(intake), new WaitCommand(1.8), new StopDrop(intake)));
    NamedCommands.registerCommand("RaiseIntake", new SequentialCommandGroup(new RaiseIntake(intake), new WaitCommand(1.75), new StopDrop(intake)));
    NamedCommands.registerCommand("StopShootCommand", new ParallelCommandGroup(new StopShootCommand(shooter, lights), new StopIntakeSeq(intake)));//this should interrupt???
    NamedCommands.registerCommand("StartShootCommand", new ParallelCommandGroup(new StartShootCommand(shooter, lights), new OutttakeCommand(intake)));
    // climber.zeroEncoders();
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                    OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                    OperatorConstants.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                    OperatorConstants.RIGHT_X_DEADBAND),
            driverXbox::getYButtonPressed,
            driverXbox::getAButtonPressed,
            driverXbox::getXButtonPressed,
            driverXbox::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation

   /*  Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() == DriverStation.Alliance.Red) {
          driveFieldOrientedDirectAngle= drivebase.driveCommand(
              () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
              () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
              () -> -driverXbox2.getRawAxis(0),
              () -> -driverXbox2.getRawAxis(1));
      }
    else {
      driveFieldOrientedDirectAngle= drivebase.driveCommand(
              () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
              () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
              () -> -driverXbox2.getRawAxis(0),
              () -> -driverXbox2.getRawAxis(1));
    }*/
driveFieldOrientedDirectAngle= drivebase.driveCommand(
              () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
              () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
              () -> -driverXbox2.getRawAxis(0),
              () -> -driverXbox2.getRawAxis(1));
    

    /*// Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation

    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> driverXbox.getRawAxis(2));*/

    driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
            () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> driverXbox.getRawAxis(2));
 drivebase.setDefaultCommand(
            !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  
     }
  public void setDefaultCommand(){
   lights.setDefaultCommand(new LightsDefault(lights));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new JoystickButton(driverXbox, 10).onTrue(new InstantCommand(drivebase::lock, drivebase));
    //new JoystickButton(driverXbox, 1).onTrue(new InstantCommand(() -> drivebase.aimAtTarget(vision).schedule(), drivebase));
    new JoystickButton(driverXbox, 3).onTrue(new SequentialCommandGroup(new RaiseIntake(intake), new WaitCommand(1.75), new StopDrop(intake)));
    new JoystickButton(driverXbox, 4).onTrue(new SequentialCommandGroup(new DropIntake(intake), new WaitCommand(2.7), new StopDrop(intake)));
    new JoystickButton(driverXbox2, 3).whileTrue(new ClimbCommand(climber));
    new JoystickButton(driverXbox2, 4).whileTrue(new DescendCommand(climber));
    new JoystickButton(driverXbox2, 1).whileTrue(new ShootCommand(shooter, lights));
    new JoystickButton(driverXbox2, 5).whileTrue(new TrapAmpSlowShoot(shooter, lights));
    new JoystickButton(driverXbox2, 2).whileTrue(new TrapAmp(shooter));
    new JoystickButton(driverXbox, 1).whileTrue(new IntakeCommand(intake, lights));
    new JoystickButton(driverXbox, 7).onTrue(new InstantCommand(drivebase::zeroGyro));
      new JoystickButton(driverXbox, 2).whileTrue(new OutttakeCommand(intake));
    new JoystickButton(driverXbox, 8).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    //return null;
    return drivebase.getAutonomousCommand("New Auto");
  }



  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
