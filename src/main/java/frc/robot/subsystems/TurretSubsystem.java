package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class TurretSubsystem 
{
    private TalonSRX TurretMotor;

    public TurretSubsystem()
    {
        // Configure the Motor

        TalonSRX TurretMotor = new TalonSRX(0);

        TurretMotor.configFactoryDefault();

        TurretMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);

        
        // -- Set relevant frame periods to be at least as fast as periodic rate
        TurretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
        TurretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

        // -- Set the peak and nominal outputs
        TurretMotor.configNominalOutputForward(0, 30);
        TurretMotor.configNominalOutputReverse(0, 30);
        TurretMotor.configPeakOutputForward(1, 30);
        TurretMotor.configPeakOutputReverse(-1, 30);

        // -- Soft limits
        TurretMotor.configForwardSoftLimitThreshold(0);
        TurretMotor.configReverseSoftLimitThreshold(0);
        TurretMotor.configForwardSoftLimitEnable(false);
        TurretMotor.configReverseSoftLimitEnable(false);


        TurretMotor.config_kF(0, 0, 30);
        TurretMotor.config_kP(0, 0, 30);
        TurretMotor.config_kI(0, 0, 30);
        TurretMotor.config_kD(0, 0, 30);

        TurretMotor.configMotionCruiseVelocity(0, 30);
        TurretMotor.configMotionAcceleration(0, 30); 
        TurretMotor.configMotionSCurveStrength(0);
 

        TurretMotor.setSelectedSensorPosition(0, 0, 30);
    }

    private double getMotorPosition() //get motor position (redundant?)
    {
        return TurretMotor.getSelectedSensorPosition();
    }


    private double RotationToEncoderTicks(double Rotations) //convert desired rotations to encoder ticks
    {
        double TicksperRotation = 4096; //encoder ticks per roation of the TalonSRX magnetic encoder

        double RequiredEncoderTicks = TicksperRotation * Rotations;

        return RequiredEncoderTicks;
    }

    private double EncoderTicksToRotation() //convert encoder ticks to rotation
    {
        double TicksperRotation = 4096;

        return getMotorPosition() / TicksperRotation;
    
    }

    public Command TurretGoToPosition(double Rotations) //set turret to specific position
    {
        return Commands.run(() ->
        {
            TurretMotor.set(ControlMode.MotionMagic,RotationToEncoderTicks(Rotations));
        }).until(() ->
        {
            return MathUtil.isNear(RotationToEncoderTicks(Rotations), getMotorPosition(), 10);
        });
    }

    public Command Command_LimelightAimTurret() //Aim turret with Limelight ( very simple, Have no idea if this works)
    {
        return Commands.run(() ->
        {
            double x = LimelightHelpers.getTX("") * 0.4;
            double ControlTarget = TurretMotor.getSelectedSensorPosition() + x;
            TurretMotor.set(ControlMode.MotionMagic, ControlTarget);
            if (EncoderTicksToRotation() > 1.5)
            {
                Commnad_TurretLimitReached(true);
            } else if (EncoderTicksToRotation() < -1.5)
            {
              Commnad_TurretLimitReached(false);
            }

        });
    }

    // will swing the turret, 360, to the other direction if the roatation limit is reached so the wires do not get destoryed
    public Command Commnad_TurretLimitReached(boolean ForwardLimit)
    {
        double ForwardLimitSwing = TurretMotor.getSelectedSensorPosition() - 4079;
        double ReverseLimitSwing = TurretMotor.getSelectedSensorPosition() + 4079;


       return Commands.run(() ->
        {

            if (ForwardLimit)
            {
                TurretMotor.set(ControlMode.MotionMagic, ForwardLimitSwing);
            }
            else
            {
                TurretMotor.set(ControlMode.MotionMagic, ReverseLimitSwing);
            }
        }).until(() ->
        {
            return MathUtil.isNear(ForwardLimit ? ForwardLimitSwing : ReverseLimitSwing, TurretMotor.getSelectedSensorPosition(), 10);
        });
    }

    public void periodic() //print rotations
    {
        SmartDashboard.putNumber("Roations", EncoderTicksToRotation());
    }
}



      
