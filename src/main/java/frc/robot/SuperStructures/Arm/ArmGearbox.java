// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SuperStructures.Arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Mechanisms.ArmGearbox.ArmGearboxIO;
import frc.robot.Mechanisms.ArmGearbox.ArmGearboxIOInputsAutoLogged;
import frc.robot.Util.Team6328.LoggedTunableNumber;

/**
 * Here is where high level coding happens
 */
public class ArmGearbox extends SubsystemBase {
  // Hardware Control
  private ArmGearboxIO io;
  private ArmGearboxIOInputsAutoLogged inputs = new ArmGearboxIOInputsAutoLogged();

  //TODO: Add the mechanism 2d: https://docs.wpilib.org/en/latest/docs/software/dashboards/glass/mech2d-widget.html

  // Tuneable numbers
  public static final LoggedTunableNumber armAngle = // Set a "Set Point" angle
      new LoggedTunableNumber("ArmGearbox/armAngle");
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("ArmGearbox/kP");
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("ArmGearbox/kI");
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("ArmGearbox/kD");
  public static final LoggedTunableNumber kIZ = // Value just for Real
      new LoggedTunableNumber("ArmGearbox/kIZ");
  public static final LoggedTunableNumber kFF = new LoggedTunableNumber("ArmGearbox/kFF");
  //public static final LoggedTunableNumber kMaxVelo = new LoggedTunableNumber("ArmGearbox/kMaxVelo");
  //public static final LoggedTunableNumber kMaxAcc = new LoggedTunableNumber("ArmGearbox/kMaxAcc");

  /** Creates a new ArmGearbox. */
  public ArmGearbox(ArmGearboxIO io) {
    System.out.println("[Init] Creating ArmGearbox");
    this.io = io;
     switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        // Set the pid values at the very start
        armAngle.initDefault(0);// To use this one create a Tuner Command
        kP.initDefault(0);
        kI.initDefault(0);
        kD.initDefault(0);
        kIZ.initDefault(0);
        kFF.initDefault(0);
        io.setPIDGains(kP.get(), kI.get(), kD.get(), kIZ.get(), kFF.get());
        break;
      case SIM:
        armAngle.initDefault(0);
        kP.initDefault(0);
        kI.initDefault(0);
        kD.initDefault(0);
        kIZ.initDefault(0);// Leave it as 0
        kFF.initDefault(0);
        io.setPIDGains(kP.get(), kI.get(), kD.get(), kIZ.get(), kFF.get());   
        break;
      default:
        break;
    }
  
    io.setBrakeMode(true);
  }
  
  public void allowMovementForTuning(){
    io.setReference(armAngle.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    //Update values if they had changed
    if(kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode()) 
    || kIZ.hasChanged(hashCode()) || kFF.hasChanged(hashCode())){
        io.setPIDGains(kP.get(), kI.get(), kD.get(), kIZ.get(), kFF.get());
    }

    //if(kMaxAcc.hasChanged(hashCode()) || kMaxVelo.hasChanged(hashCode())){
    //    io.setSmartMotionGains(kMaxVelo.get(), kMaxAcc.get());
    //}
  }
}
