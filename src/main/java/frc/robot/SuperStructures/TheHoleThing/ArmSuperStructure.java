// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SuperStructures.TheHoleThing;

import frc.robot.Mechanisms.ArmGearbox.ArmGearbox;
import frc.robot.Mechanisms.BeltMechanism.BeltMechanism;

/** Add your docs here. */
public class ArmSuperStructure {
    private ArmGearbox armGearbox;
    private BeltMechanism beltMechanism;
    
    public ArmSuperStructure(ArmGearbox armGearbox, BeltMechanism beltMechanism) {
        this.armGearbox = armGearbox;
        this.beltMechanism = beltMechanism;
    }

    public void setRollersVoltage(){

    }

    public void setArmPos(){}
}
