package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX intakeMotor = new TalonSRX(Constants.MotorConstants.INTAKE_ID);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.MotorConstants.INTAKE_LIMIT_SWITCH_PORT);
    public TalonSRX armL;
    public TalonSRX armR;

    public void intake() {
        // If the limit switch is not engaged, run the motor forward, intake the ring
        if (!limitSwitch.get()) {
            intakeMotor.set(ControlMode.PercentOutput, 1.0);
        } else {
            intakeMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }
    public double getArmPosition(TalonSRX arm){
        return arm.getSelectedSensorPosition();
    }
    public void reverse() {
        // If the limit switch is not engaged, run the motor in reverse
        if (!limitSwitch.get()) {
            intakeMotor.set(ControlMode.PercentOutput, -1.0);
        } else {
            intakeMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void stop() {
        // Stop the motor
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public IntakeSubsystem() {
        armL=new TalonSRX(Constants.MotorConstants.ARM_L_ID);
        armR=new TalonSRX(Constants.MotorConstants.ARM_R_ID);
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
    }
    @Override
    public void periodic(){

    }

}

