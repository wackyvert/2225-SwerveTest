package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.MotorConstants.INTAKE_ID, MotorType.kBrushless);
    //private final DigitalInput limitSwitch = new DigitalInput(Constants.MotorConstants.INTAKE_LIMIT_SWITCH_PORT);
    public TalonSRX pivot=new TalonSRX(Constants.MotorConstants.ARM_R_ID);
       
       
    
    
    public double armSyncOutput;
    public void intake() {
        // If the limit switch is not engaged, run the motor forward, intake the ring
        
            intakeMotor.set(-.85);
       
    }
    public double getArmPosition(){
        return pivot.getSelectedSensorPosition()/2048;
    }
    public void dropIntake(){
         pivot.set(ControlMode.PercentOutput, .4);
            
     }
    public void raiseIntake(){
      pivot.set(ControlMode.PercentOutput, -.4);
        
    }
    
    public void reverse() {
        // If the limit switch is not engaged, run the motor in reverse
        
            intakeMotor.set( .4);
        
            
    }

    public void stop() {
        // Stop the motor
        intakeMotor.set(0.0);
       pivot.set(ControlMode.PercentOutput, 0);
    }

    public IntakeSubsystem() {
       
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
    }
    public void zeroEncoders(){
        pivot.setSelectedSensorPosition(0);
    }
    @Override
    public void periodic(){
            SmartDashboard.putNumber("Intake arm", getArmPosition()/1000);
    }

}

