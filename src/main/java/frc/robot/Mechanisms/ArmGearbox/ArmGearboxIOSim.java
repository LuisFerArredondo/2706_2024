// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.ArmGearbox;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;

/** Add your docs here. */
public class ArmGearboxIOSim implements ArmGearboxIO{
    private SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 30,5, 1, 0, ((3 * Math.PI) / 4), true, 0, VecBuilder.fill((2 * Math.PI) / 1024));
    private DutyCycleEncoderSim absouluteEncoderSim = new DutyCycleEncoderSim(10);
    private PIDController m_controller = new PIDController(0, 0, 0);
    private double ffVolts = 0.0;
    private double appliedVolts = 0.0;

    /*
     TODO: add the Single Jointed Arm Sim, add a normal pid with a normal feedforward calculation:
        "SetVoltage(pidOutput + FeedForwardOutput)"
        https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/physics-sim.html
     * 
    */    

    @Override
    public void updateValues(ArmGearboxIOValues inputs) {

        armSim.update(0.02);

        inputs.armAbsolutePositionRad = armSim.getAngleRads();
        inputs.armAppliedVolts = appliedVolts;
        inputs.armCurrentAmps = new double[]{armSim.getCurrentDrawAmps()};
        inputs.armTempCelcius = new double[]{0.0};
    }

    @Override
    public void setBrakeMode(boolean brakeEnabled) {
        Logger.recordOutput("ArmSim/idleMode", brakeEnabled ? "Brake" : "Coast");
    }

    @Override
    public void setPIDGains(double p, double i, double d, double iz, double ff) {
        m_controller.setPID(p, i, d);
        ffVolts = ff;
    }

    @Override
    public void setReference(double value) {
        double setPoint = Units.radiansToDegrees(value);
        m_controller.setSetpoint(setPoint);
        appliedVolts = MathUtil.clamp(m_controller.calculate(armSim.getAngleRads()) + ffVolts, -12, 12);
        armSim.setInputVoltage(appliedVolts);

    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        armSim.setInputVoltage(volts);
    }
}
