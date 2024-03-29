package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {




    // Shooter motors
    private final CANSparkMax shooterMotor1 = new CANSparkMax(Constants.MotorConstants.SHOOTER_L_ID, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax shooterMotor2 = new CANSparkMax(Constants.MotorConstants.SHOOTER_R_ID, CANSparkMax.MotorType.kBrushless);

    // Feeder motor
    private final TalonSRX feederMotor = new TalonSRX(Constants.MotorConstants.SHOOTER_LOADER_ID);

    // Encoders


    // PID controllers
    private final PIDController shooterPIDController1 = new PIDController(0,0,0);
    private final PIDController shooterPIDController2 = new PIDController(0,0,0);

    public ShooterSubsystem() {
        shooterMotor2.setInverted(true);
        shooterMotor1.setInverted(true);
        shooterPIDController1.setTolerance(100, 100);
        shooterPIDController1.setP(Constants.PIDFConstants.ShooterPIDConstants.P);
        shooterPIDController1.setI(Constants.PIDFConstants.ShooterPIDConstants.I);
        shooterPIDController1.setD(Constants.PIDFConstants.ShooterPIDConstants.D);
        shooterPIDController2.setTolerance(100, 100);
        shooterPIDController2.setP(Constants.PIDFConstants.ShooterPIDConstants.P);
        shooterPIDController2.setI(Constants.PIDFConstants.ShooterPIDConstants.I);
        shooterPIDController2.setD(Constants.PIDFConstants.ShooterPIDConstants.D);
    }


    public boolean readyToFire(CANSparkMax shooterMotor)
    {
        double tolerance = 100;
        double actualVelocity = getEncoderSpeed(shooterMotor);
        double targetVelocity = Constants.PIDFConstants.ShooterPIDConstants.SHOOTER_SETPOINT;
        if (Math.abs(actualVelocity - targetVelocity) <= tolerance) {
            return true;
        } else {
            return false;
            // Velocity is out of range
        }
    }
    public void shoot() {
        shooterMotor1.set(-.9);
        shooterMotor2.set(.9);
        feed();
    }

    public void stopShooter() {
        shooterMotor1.set(0.0);
        shooterMotor2.set(0.0);
        feederMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public double getEncoderSpeed(CANSparkMax motor){
       return motor.getEncoder().getVelocity();
    }
    public void feed(){
        feederMotor.set(ControlMode.PercentOutput, .5);
    }

    @Override
    public void periodic(){

    }

}

