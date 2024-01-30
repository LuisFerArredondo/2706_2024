// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SuperStructures.Arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Mechanisms.ArmGearbox.ArmGearboxIO;
import frc.robot.Mechanisms.ArmGearbox.ArmGearboxIOValuesAutoLogged;
import frc.robot.Util.Team6328.LoggedTunableNumber;

/**
 * Here is where high level coding happens
 */
public class ArmGearbox extends SubsystemBase {
  // Hardware Control
  private ArmGearboxIO io;
  private ArmGearboxIOValuesAutoLogged inputs = new ArmGearboxIOValuesAutoLogged();

  //TODO: Add the mechanism 2d: 
  // https://docs.wpilib.org/en/latest/docs/software/dashboards/glass/mech2d-widget.html
  Mechanism2d arm = new Mechanism2d(10, 5, new Color8Bit(Color.kPurple));
  MechanismRoot2d root = arm.getRoot("Arm", 0, 0);

  // Tuneable numbers
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("ArmGearbox/kP");
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("ArmGearbox/kI");
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("ArmGearbox/kD");
  public static final LoggedTunableNumber kIZ = // Value just for Real
      new LoggedTunableNumber("ArmGearbox/kIZ");
  public static final LoggedTunableNumber kFF = new LoggedTunableNumber("ArmGearbox/kFF");

  /** Creates a new ArmGearbox. */
  public ArmGearbox(ArmGearboxIO io) {
    System.out.println("[Init] Creating ArmGearbox");
    this.io = io;
     switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        // Set the pid values at the very start
        kP.initDefault(0);
        kI.initDefault(0);
        kD.initDefault(0);
        kIZ.initDefault(0);
        kFF.initDefault(0);
        
        io.setPIDGains(kP.get(), kI.get(), kD.get(), kIZ.get(), kFF.get());
        break;
      case SIM:
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
  
  public void setTuningSetPoint(double setPoint){
    io.setReference(setPoint);
  }

  public void stop(){
    io.setVoltage(0);
  }

  @Override
  public void periodic() {
    io.updateValues(inputs);
    Logger.processInputs(getName(), inputs);

    //Update values if they had changed
    if(kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode()) 
    || kIZ.hasChanged(hashCode()) || kFF.hasChanged(hashCode())){
        io.setPIDGains(kP.get(), kI.get(), kD.get(), kIZ.get(), kFF.get());
    }
  }
}
