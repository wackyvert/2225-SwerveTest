package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.MotorConstants.INTAKE_ID, MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.MotorConstants.INTAKE_LIMIT_SWITCH_PORT);
    public TalonSRX armL;
    public double intakeLowerSetpoint=100000000;//need to set!!
    public double intakeUpperSetpoint=1;
    public TalonSRX armR;
    public double armSyncOutput;
    public void intake() {
        // If the limit switch is not engaged, run the motor forward, intake the ring
        
            intakeMotor.set(.7);
       
    }
    public double getArmPosition(){
        return armR.getSelectedSensorPosition()/2048;
    }
    public void dropIntake(){
        if (getArmPosition()<intakeLowerSetpoint) {
            armSyncOutput=-0.2;
            armL.set(ControlMode.PercentOutput, armSyncOutput);
            armR.set(ControlMode.PercentOutput, armSyncOutput);
        }
        else{
            armSyncOutput=0;
            armL.set(ControlMode.PercentOutput, armSyncOutput);
            armR.set(ControlMode.PercentOutput, armSyncOutput);
        }
    }
    public void raiseIntake(){
        if (getArmPosition()>intakeUpperSetpoint) {
            armSyncOutput=0.2;
            armL.set(ControlMode.PercentOutput, armSyncOutput);
            armR.set(ControlMode.PercentOutput, armSyncOutput);
        }
        else{
            armSyncOutput=0;
            armL.set(ControlMode.PercentOutput, armSyncOutput);
            armR.set(ControlMode.PercentOutput, armSyncOutput);
        }
    }
    
    public void reverse() {
        // If the limit switch is not engaged, run the motor in reverse
        if (!limitSwitch.get()) {
            intakeMotor.set( -.2);
        } else {
            intakeMotor.set(0.0);
        }
    }

    public void stop() {
        // Stop the motor
        intakeMotor.set(0.0);
        armL.set(ControlMode.PercentOutput, 0);
        armR.set(ControlMode.PercentOutput, 0);
    }

    public IntakeSubsystem() {
        armL=new TalonSRX(Constants.MotorConstants.ARM_L_ID);
        armR=new TalonSRX(Constants.MotorConstants.ARM_R_ID);
       zeroEncoders();
        
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
    }
    public void zeroEncoders(){
        armR.setSelectedSensorPosition(0);
    }
    @Override
    public void periodic(){

    }

}

