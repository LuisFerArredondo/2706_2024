// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltMechanism extends SubsystemBase {
    private byte deviceID = 1;
    private CANSparkMax top_roller;
    private SparkPIDController sparkMaxPID;
    private RelativeEncoder sparkMaxRelativeEncoder;
    private int smartMotionSlot = 0;
  
  /** Creates a new BeltMechanism. */
  public BeltMechanism() {
        top_roller = new CANSparkMax(deviceID, MotorType.kBrushless);
        top_roller.restoreFactoryDefaults();
        
        //Soft limit params
        top_roller.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        top_roller.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        top_roller.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
        top_roller.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

        sparkMaxPID = top_roller.getPIDController();
        sparkMaxRelativeEncoder = top_roller.getEncoder();    
        sparkMaxRelativeEncoder.setAverageDepth(2);
        sparkMaxRelativeEncoder.setPositionConversionFactor(1);
        sparkMaxRelativeEncoder.setVelocityConversionFactor(1);
        sparkMaxRelativeEncoder.setPosition(0);

        //Voltage compensation
        top_roller.enableVoltageCompensation(12); //Check this values
        top_roller.setSmartCurrentLimit(30); //Units in AMPS, should not be more than 40
        top_roller.setCANTimeout(250); //Units in miliseconds

        //PID values
        sparkMaxPID.setP(0);//check this values
        sparkMaxPID.setI(0);//check this values
        sparkMaxPID.setD(0);//check this values
        sparkMaxPID.setIZone(0);//check this values
        sparkMaxPID.setFF(12);//check this values
        sparkMaxPID.setOutputRange(1, -1);

        //Smart motion parameters 
        sparkMaxPID.setSmartMotionMaxVelocity(2000, smartMotionSlot);//check this values
        sparkMaxPID.setSmartMotionMinOutputVelocity(0, smartMotionSlot);//check this values
        sparkMaxPID.setSmartMotionMaxAccel(1500, smartMotionSlot);
        sparkMaxPID.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);//check this values
  }

  //Voltage applied to the motor
  public void setVoltage(double volts){
    top_roller.setVoltage(volts);
  }

  /**
   * This method uses the sparkMax pid
   */
  public void setReference(double value, ControlType type, byte PID){
    sparkMaxPID.setReference(value, type, PID, value);
  }

  public double getEncoderPostion(){
    return sparkMaxRelativeEncoder.getPosition();
  }

  public double getEncoderVelocity(){
    return sparkMaxRelativeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
