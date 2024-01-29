// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SuperStructures.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmTuningCommand extends Command {
  private ArmGearbox armGearbox;

  /** Creates a new ArmTuningCommand. */
  public ArmTuningCommand(ArmGearbox armGearbox) {
    this.armGearbox = armGearbox;

    addRequirements(armGearbox);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armGearbox.allowMovementForTuning();
  }
}
