// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.DebuggingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructures.Arm.ArmGearbox;

public class ArmTuner extends Command {
  private ArmGearbox armGearbox;
  private double setPoint;

  /** Creates a new ArmTuner. */
  public ArmTuner(ArmGearbox armGearbox, double setPoint) {
    this.armGearbox = armGearbox;
    this.setPoint = setPoint;

    addRequirements(armGearbox);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armGearbox.setTuningSetPoint(setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armGearbox.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
