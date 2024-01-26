// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.HashMap;
import java.util.Map;

/**
 *  Document to avoid having to write the same code for the spark max every time
 * 
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

 */
public class SmartSparkMaxConstants {
    protected HashMap<String, Double> values = new HashMap<>();

    public SmartSparkMaxConstants(double kP, double kI, double kD, double kIz, double kFF, double allowedErr){
        this(kP, kI, kD, kIz, kFF, 1, -1, 5700, 2000, 0, 1500, allowedErr);
    }

    public SmartSparkMaxConstants(double kP, double kI, double kD, double kIz, double kFF, double kMaxOutput,
        double kMinOutput, double maxRPM, double maxVel, double minVel, double maxAcc, double allowedErr) {
    }
    /**
     * 
     * @param kP P term
     * @param kI I term
     * @param kD D term
     * @param kIz I zone 
     * @param kFF Feed Forward
     * @param kMaxOutput Max Output (-1, 0, 1)
     * @param kMinOutput
     * @param maxRPM
     * @param maxVel
     * @param minVel
     * @param maxAcc
     * @param allowedErr
     * @param enableForwardSoftLimit 0 -> false, 1 -> true
     * @param enableReverseSoftLimit
     * @param forwardSoftLimit
     * @param reverseSoftLimit
     */
    public SmartSparkMaxConstants(double kP, double kI, double kD, double kIz, double kFF, double kMaxOutput,
            double kMinOutput, double maxRPM, double maxVel, double minVel, double maxAcc, double allowedErr, double enableForwardSoftLimit, double enableReverseSoftLimit, double forwardSoftLimit, double reverseSoftLimit) {
            
        values.put("kP", kP);
        values.put("kI", kI);
        values.put("kD", kD);
        values.put("kIz", kIz);
        values.put("kFF", kFF);
        values.put("kMaxOutput", kMaxOutput);
        values.put("kMinOutput", kMinOutput);
        values.put("maxRPM", maxRPM);
        values.put("maxVel", maxVel);
        values.put("minVel", minVel);
        values.put("maxAcc", maxAcc);
        values.put("allowedErr", allowedErr);        
        values.put("enableForwardSoftLimit", enableForwardSoftLimit);        
        values.put("enableReverseSoftLimit", enableReverseSoftLimit);        
        values.put("forwardSoftLimit", forwardSoftLimit);        
        values.put("reverseSoftLimit", reverseSoftLimit);        
    }

    public void iterateSparkConstants(){
        for (Map.Entry<String,Double> entry: values.entrySet()) {
            System.out.println(entry.getKey() + ": " + entry.getValue());
        }
    }
    

}
