// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.ArmGearbox;

import org.littletonrobotics.junction.AutoLog;

/** In this class we define which methods we will use to interact with the hardware*/
public interface ArmGearboxIO {
    
    @AutoLog
    public static class ArmGearboxIOValues{
        public double armAbsolutePositionRad = 0.0;//Absolute encoder pos
        public double armAppliedVolts = 0.0;//voltage applied to the spark

        public double[] armCurrentAmps = new double[] {};//An array in case there's two motors
        public double[] armTempCelcius = new double[] {};//An array in case there's two motors
    }

    /*update logget inputs  */
    public default void updateValues(ArmGearboxIOValues inputs){}

    /*set the arm motor's brake ON or OFF */
    public default void setBrakeMode(boolean brakeEnabled){}

    /*Modify PID gains for tuning */
    public default void setPIDGains(double p, double i, double d, double iz, double ff){}

    public default void setReference(double value){}

    public default void setVoltage(double volts){}
    //public default void resetInternalEncoder(){}
}
