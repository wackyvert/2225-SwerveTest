// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final class MotorConstants{
    public static int CLIMBER_ID=0;
    public static final int SHOOTER_L_ID = 1;
    public static final int SHOOTER_R_ID = 1;
    public static final int SHOOTER_LOADER_ID=1;
    public static final int TRAP_MOTOR_ID=1;
    public static final int INTAKE_ID = 2;
    public static final int INTAKE_LIMIT_SWITCH_PORT=0;


  }
  public static final class PIDFConstants {
    public static final class ClimberPIDConstants {
      public static double P = 0.002;
      public static double I = 0;
      public static double D = 0;

    }
    public static final class ShooterPIDConstants {
      public static double P = 0.002;
      public static double I = 0;
      public static double D = 0;

      public static double SHOOTER_SETPOINT;
    }
  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.0, 0, 0.00);
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.2;
    public static final double LEFT_Y_DEADBAND = 0.2;
    public static final double RIGHT_X_DEADBAND = 0.2;
    public static final double RIGHT_Y_DEADBAND= 0.2;
    public static final double TURN_CONSTANT = 0.75;
  }
}
