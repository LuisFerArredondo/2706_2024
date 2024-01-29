// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.ArmGearbox;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmGearboxIOSim implements ArmGearboxIO{
    private SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 30,5, 1, 0, ((3 * Math.PI) / 4), true, 0, VecBuilder.fill((2 * Math.PI) / 1024));
    private DutyCycleEncoderSim absouluteEncoderSim = new DutyCycleEncoderSim(10);
    private PIDController m_controller;
    private double armAppliedVolts;

    /*
     TODO: add the Single Jointed Arm Sim, add a normal pid with a normal feedforward calculation:
        "SetVoltage(pidOutput + FeedForwardOutput)"
        https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/physics-sim.html
     * 
    */
    public ArmGearboxIOSim(){
        
    }

    @Override
    public void setBrakeMode(boolean brakeEnabled) {
        // TODO Auto-generated method stub
        ArmGearboxIO.super.setBrakeMode(brakeEnabled);
    }

    @Override
    public void setPIDGains(double p, double i, double d, double iz, double ff) {
        // TODO Auto-generated method stub
        ArmGearboxIO.super.setPIDGains(p, i, d, iz, ff);
    }

    @Override
    public void setReference(double value) {
        // TODO Auto-generated method stub
        ArmGearboxIO.super.setReference(value);
    }

    @Override
    public void updateInputs(ArmGearboxIOInputs inputs) {
        // TODO Auto-generated method stub
        ArmGearboxIO.super.updateInputs(inputs);
    }
}
