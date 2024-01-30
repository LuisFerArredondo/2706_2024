// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.ArmGearbox;

import static frc.robot.Util.Team6328.CleanSparkMaxValue.cleanSparkMaxValue;

import org.littletonrobotics.junction.Logger;

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
    private CANSparkMax m_motor = new CANSparkMax(1, MotorType.kBrushless);

    private SparkPIDController m_pidController;
    private AbsoluteEncoder absoluteEncoder;
    private boolean setReverse = true;

    //Constants
    private int SMART_MOTION_DEFAULT_SLOT = 0;

    //TODO: add the spark max brun manager
    public ArmGearboxIOSparkMax(){
        System.out.println("[Init] Creating CubeIntakeIOSparkMax");
        m_motor.restoreFactoryDefaults();
        
        //Soft limit params
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        m_motor.setInverted(setReverse);
        m_motor.setCANTimeout(500); //Units in miliseconds
        m_motor.setIdleMode(IdleMode.kCoast);

        m_pidController = m_motor.getPIDController();
        absoluteEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setAverageDepth(2);//check if 1 would work better
        
        //Voltage compensation
        m_motor.enableVoltageCompensation(12); //Check this values
        m_motor.setSmartCurrentLimit(40); //Units in AMPS, should not be more than 40 check with 
        //Hardware to which port on the pdh the motor is connected to
  
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
    public void setReference(double value){
        double setpoint = Units.degreesToRotations(value);
        m_pidController.setReference(setpoint, ControlType.kSmartMotion);

        Logger.recordOutput("ArmGearbox/SetPointPosition", value);
    }

    @Override
    public void updateValues(ArmGearboxIOValues inputs) {
        inputs.armAbsolutePositionRad = 
            MathUtil.angleModulus(
                Units.rotationsToRadians(absoluteEncoder.getPosition() * (setReverse ? -1 : 1)));

        inputs.armAppliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        
        inputs.armCurrentAmps = new double[]{m_motor.getOutputCurrent()};
        
        inputs.armTempCelcius = new double[]{m_motor.getMotorTemperature()};
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
    public void setVoltage(double volts) {
        m_motor.setVoltage(0);
    }

}
