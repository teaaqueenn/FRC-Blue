package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private TalonFX motorFx;

    private MotionMagicVoltage m_mmReq;
    private TalonFXConfiguration cfg;
    private MotionMagicConfigs mm;

    private double setpoint;

    private static Elevator elevatorSys = new Elevator();

    private Elevator(){
        motorFx = new TalonFX(Constants.ElevatorConstants.MOTOR_ID);
        motorFx.setNeutralMode(NeutralModeValue.Brake);

        m_mmReq = new MotionMagicVoltage(0);
        cfg = new TalonFXConfiguration();
        mm = cfg.MotionMagic;
        setpoint = Constants.ElevatorConstants.HOME_POSITION;

        cfg.Slot0.kS = Constants.ElevatorConstants.kS;
        cfg.Slot0.kV = Constants.ElevatorConstants.kV;
        cfg.Slot0.kG = Constants.ElevatorConstants.kG;
        cfg.Slot0.kP = Constants.ElevatorConstants.kP;
        cfg.Slot0.kI = Constants.ElevatorConstants.kI;
        cfg.Slot0.kD = Constants.ElevatorConstants.kD;

        mm.MotionMagicCruiseVelocity = Constants.ElevatorConstants.mmVelo;
        mm.MotionMagicAcceleration = Constants.ElevatorConstants.mmAcc;
        mm.MotionMagicJerk = Constants.ElevatorConstants.mmJerk;
        /**
         *Enable For Lim
         Enable For Lim Pos Res
         Inv motor
         */
        mm = cfg.MotionMagic;

        cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;

        motorFx = new TalonFX(Constants.ElevatorConstants.MOTOR_ID);
    }

    public void pid(double input){
        setpoint = input;
        motorFx.setControl(m_mmReq.withPosition(setpoint));
        SmartDashboard.putNumber("Elevator Position", motorFx.getPosition().getValueAsDouble());
    }

    public boolean stopPID(){
        if(-1 < motorFx.getPosition().getValue() - setpoint || motorFx.getPosition().getValue() - setpoint <= 1){
            return true;
        }else{
            return false;
        }
    }

    public void setpoint(double input){
        setpoint = input;
    }

    public static Elevator getSystem(){
        return elevatorSys;
    }

    public void stop(){
        motorFx.stopMotor();
    }
}
