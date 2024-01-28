// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.ArmGearbox;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmGearboxBase extends SubsystemBase {
  private CANSparkMax top_roller;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  
  //Constants
  private byte DEVICE_ID = 1;
  private int SMART_MOTION_DEFAULT_SLOT = 0;
  private double GEAR_RATIO = 0.0;
  
  /** Creates a new BeltMechanism. */
  public ArmGearboxBase() {
        top_roller = new CANSparkMax(DEVICE_ID, MotorType.kBrushless);
        top_roller.restoreFactoryDefaults();
        
        //Soft limit params
        top_roller.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        top_roller.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        top_roller.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
        top_roller.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        top_roller.setCANTimeout(250); //Units in miliseconds

        m_pidController = top_roller.getPIDController();
        m_encoder = top_roller.getEncoder();    

        //Voltage compensation
        top_roller.enableVoltageCompensation(12); //Check this values
        top_roller.setSmartCurrentLimit(30); //Units in AMPS, should not be more than 40

        //PID values
        m_pidController.setP(0);//check this values
        m_pidController.setI(0);//check this values
        m_pidController.setD(0);//check this values
        m_pidController.setIZone(0);//check this values
        m_pidController.setFF(12);//check this values
        m_pidController.setOutputRange(1, -1);

        //Smart motion parameters 
        m_pidController.setSmartMotionMaxVelocity(2000, SMART_MOTION_DEFAULT_SLOT);//check this values
        m_pidController.setSmartMotionMinOutputVelocity(0, SMART_MOTION_DEFAULT_SLOT);//check this values
        m_pidController.setSmartMotionMaxAccel(1500, SMART_MOTION_DEFAULT_SLOT);
        m_pidController.setSmartMotionAllowedClosedLoopError(0, SMART_MOTION_DEFAULT_SLOT);//check this values
  }

  //Voltage applied to the motor
  public void setVoltage(double volts){
    top_roller.setVoltage(volts);
  }

  /**
   * This method uses the sparkMax pid
   */
  public void setReference(double value, ControlType type, byte PID){
    m_pidController.setReference(value, type, PID, value);
  }

  public double getRelativePosition(){
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
