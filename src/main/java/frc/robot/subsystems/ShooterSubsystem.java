package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {


    private final TalonSRX trapMotor = new TalonSRX(15);

    // Shooter motors
    private final CANSparkFlex shooterMotor1 = new CANSparkFlex(Constants.MotorConstants.SHOOTER_L_ID, CANSparkFlex.MotorType.kBrushless);
    private final CANSparkFlex shooterMotor2 = new CANSparkFlex(Constants.MotorConstants.SHOOTER_R_ID, CANSparkFlex.MotorType.kBrushless);

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


    public boolean readyToFire(CANSparkFlex shooterMotor)
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
    public void runTrapMotor(){
        trapMotor.set(ControlMode.PercentOutput, 0.7);
    }
    public void stopTrapMotor(){
             trapMotor.set(ControlMode.PercentOutput, .0);
   
    } 
    public void shoot() {
        shooterMotor1.set(-.9);
        shooterMotor2.set(.9);
        
    }
    public void shootSlow() {
        shooterMotor1.set(-.1);
        shooterMotor2.set(.1);
        
    }

    public void stopShooter() {
        shooterMotor1.set(0.0);
        shooterMotor2.set(0.0);
    }

    public double getEncoderSpeed(CANSparkFlex motor){
       return motor.getEncoder().getVelocity();
    }
    

    @Override
    public void periodic(){

    }

}

