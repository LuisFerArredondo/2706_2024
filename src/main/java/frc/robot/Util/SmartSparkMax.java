// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class SmartSparkMax {
    private byte deviceID = 1;
    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private SmartSparkMaxConstants constants;
    private int smartMotionSlot = 0;
    /*
     * TODO: 
     * -Add SoftLimits
     * -Add Current Limits
     * -Add enable Voltage Compensation
     * -Add setCurrent
     * -Delete the pid and encoder thing, lets leave this as just the pure motor
     * - add getter for the encoders
    */
    public SmartSparkMax(byte devideID, SmartSparkMaxConstants constants) {
        this.constants = constants;
        this.deviceID = devideID;
        
        m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        
        try {
            // set PID coefficients
            m_pidController.setP(constants.values.get("kP"));
            m_pidController.setI(constants.values.get("kI"));
            m_pidController.setD(constants.values.get("kD"));
            m_pidController.setIZone(constants.values.get("kIz"));
            m_pidController.setFF(constants.values.get("kFF"));
            m_pidController.setOutputRange(constants.values.get("kMinOutput"), constants.values.get("kMaxOutput"));
        } catch (Exception e) {
            System.out.println("program not able to fin values in map, pid, Ex: " + e);
        }
        
        try {
            m_pidController.setSmartMotionMaxVelocity(constants.values.get("maxVel"), smartMotionSlot);
            m_pidController.setSmartMotionMinOutputVelocity(constants.values.get("minVel"), smartMotionSlot);
            m_pidController.setSmartMotionMaxAccel(constants.values.get("maxAcc"), smartMotionSlot);
            m_pidController.setSmartMotionAllowedClosedLoopError(constants.values.get("allowedErr"), smartMotionSlot);
        } catch (Exception e) {
            System.out.println("program not able to fin values in map, smart motion, Ex: " + e);
        }
    }

    public void setVelocity(double rpm){
        m_pidController.setReference(rpm, ControlType.kVelocity, 0);
    }

    //TODO: check this logic
    public void setVoltage(double volts){
        m_pidController.setReference((volts > 12)? 12 : volts, ControlType.kVoltage, 0);
    }   
  
}
