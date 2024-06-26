package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    //Encoder stops - Dividing output by 2048 (talonFx cpr) to have more reasonable numbers
    final double LOWER_STOP_POINT= Constants.MotorConstants.LOWER_STOP_POINT;
    final double UPPER_STOP_POINT=Constants.MotorConstants.UPPER_STOP_POINT;
//this is intentionally set ridiculously low as untested mechanism dont want to break before we can figure out what speed is best
    PIDController climberPID = new PIDController(Constants.PIDFConstants.ClimberPIDConstants.P, Constants.PIDFConstants.ClimberPIDConstants.I, Constants.PIDFConstants.ClimberPIDConstants.D );

    public TalonFX climber = new TalonFX(Constants.MotorConstants.CLIMBER_ID);
       
    
    // Motor controllers

    public double getClimberPosition(){
        return climber.getPosition().getValueAsDouble();
    }
    
    public void zeroEncoders(){
        //talonsrx is phoenix 5 and talonfx is phoenix 6
        
        climber.setPosition(0);
    }

    public void climb() {
        if (getClimberPosition()<UPPER_STOP_POINT){
        climber.set(.3);
        } else {
            stop();
        }
    }

    public void descend() {
        if (getClimberPosition()>LOWER_STOP_POINT){
            climber.set(-.3);
        } else {
            stop();
        }
    }

    // Stops the climber
    public void stop() {
        climber.set(0.0);
    }
    public ClimberSubsystem() {
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();
      //  climberConfig.Feedback.SensorToMechanismRatio=(1.0/2048);
        climberConfig.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;
        climber.setNeutralMode(NeutralModeValue.Brake);
        climber.getConfigurator().apply(climberConfig);



    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("elevator encoder", climber.getPosition().getValueAsDouble());
    }

}

