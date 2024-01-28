// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.ArmGearbox;

import static frc.robot.Util.Team6328.CleanSparkMaxValue.cleanSparkMaxValue;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

/** Add your docs here. */
public class ArmGearboxIOSparkMax implements ArmGearboxIO{
    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;
    private RelativeEncoder internalEncoder;
    //private AbsoluteEncoder absoluteEncoder;
    //private DutyCycleEncoder absoluteEncoder;
    private boolean internalEncoderReverse = true;

    //Constants
    private byte DEVICE_ID = 1;
    private int SMART_MOTION_DEFAULT_SLOT = 0;
    private double GEAR_RATIO = 0.0;

    public ArmGearboxIOSparkMax(){
        System.out.println("[Init] Creating CubeIntakeIOSparkMax");

        switch(Constants.getRobot()){
            case ROBOT_2024:
                    m_motor = new CANSparkMax(DEVICE_ID, MotorType.kBrushless);

                    //absoluteEncoder = new DutyCycleEncoder(0);//Check this value
                    //absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
                    //absoluteEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
                    //absoluteEncoder.setAverageDepth(2);//check if 1 would work better
                break;
            default:
            throw new RuntimeException("Invalid robot for ArmGearboxIOSparkMax!");

        }

        m_motor = new CANSparkMax(DEVICE_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        
        //Soft limit params
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        m_motor.setInverted(internalEncoderReverse);
        m_motor.setCANTimeout(250); //Units in miliseconds

        m_pidController = m_motor.getPIDController();
        internalEncoder = m_motor.getEncoder();    
        
        m_motor.setIdleMode(IdleMode.kCoast);
        //Voltage compensation
        m_motor.enableVoltageCompensation(12); //Check this values
        m_motor.setSmartCurrentLimit(40); //Units in AMPS, should not be more than 40 check with 
        //Hardware to which port on the pdh the motor is connected to

        internalEncoder.setPosition(0.0);
        internalEncoder.setMeasurementPeriod(10);
        internalEncoder.setAverageDepth(2);
  
        //PID values
        m_pidController.setP(0);//check this values
        m_pidController.setI(0);//check this values
        m_pidController.setD(0);//check this values
        m_pidController.setIZone(0);//check this values
        m_pidController.setFF(12);//check this values
        
        m_pidController.setOutputRange(-1, 1);
        
        //Smart motion parameters 
        m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, SMART_MOTION_DEFAULT_SLOT);
        m_pidController.setSmartMotionMaxVelocity(2000, SMART_MOTION_DEFAULT_SLOT);//check this values
        m_pidController.setSmartMotionMinOutputVelocity(0, SMART_MOTION_DEFAULT_SLOT);//check this values
        m_pidController.setSmartMotionMaxAccel(1500, SMART_MOTION_DEFAULT_SLOT);
        m_pidController.setSmartMotionAllowedClosedLoopError(0, SMART_MOTION_DEFAULT_SLOT);//check this values

    }

    @Override
    public void setReference(double value, ControlType controlType){
        m_pidController.setReference(value, controlType);
    }

    @Override
    public void updateInputs(ArmGearboxIOInputs inputs) {
        //inputs.armAbsolutePositionRad = 
        //    MathUtil.angleModulus(
        //        Units.rotationsToRadians(absoluteEncoder.getPosition() * (internalEncoderReverse ? -1 : 1)));
        
        inputs.armInternalPositionRad = 
            cleanSparkMaxValue(
                inputs.armInternalPositionRad, Units.rotationsToRadians(internalEncoder.getPosition()));//TODO: Add the gear ratio
        
        inputs.armInternalVelocityRadPerSec = 
            cleanSparkMaxValue(
                inputs.armInternalVelocityRadPerSec, Units.rotationsPerMinuteToRadiansPerSecond(internalEncoder.getPosition()));//TODO: Add the gear ratio
        
        inputs.armAppliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        
        inputs.armCurrentAmps = new double[]{m_motor.getOutputCurrent()};
        
        inputs.armTempCelcius = new double[]{m_motor.getMotorTemperature()};
    }

    @Override
    public void setArmVoltage(double volts) {
       m_motor.setVoltage(volts);
    }    

    @Override
    public void setBrakeMode(boolean brakeEnabled) {
        m_motor.setIdleMode(brakeEnabled ? IdleMode.kBrake: IdleMode.kCoast);
    }

    @Override
    public void setPIDGains(double p, double i, double d, double iz, double ff) {
        m_pidController.setP(p);//check this values
        m_pidController.setI(i);//check this values
        m_pidController.setD(d);//check this values
        m_pidController.setIZone(iz);//check this values
        m_pidController.setFF(ff);//check this values
    }

    @Override
    public void setSmartMotionGains(double maxVel, double maxAcc){
        m_pidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_DEFAULT_SLOT);//check this values
        m_pidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_DEFAULT_SLOT);
       
    }

}
