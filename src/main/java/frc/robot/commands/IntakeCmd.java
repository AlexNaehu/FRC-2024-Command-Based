// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends CommandBase {
  /** Creates a new IntakeCmd. */

  private final IntakeSubsystem intakeSubsystem;
  private final boolean open;
  private final double power;

  public IntakeCmd(IntakeSubsystem intakeSubsystem, boolean open, double power) {
    this.intakeSubsystem = intakeSubsystem;
    this.open = open;
    this.power = power;
    addRequirements(intakeSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("IntakeCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (open){
        intakeSubsystem.intake(power);
      }
      else{
        intakeSubsystem.output(power);
      }
    }
  
  @Override
  public void end(boolean interrupted) {
      System.out.println("IntakeCmd ended!");
  }

  

  @Override
  public boolean isFinished() {
    return false;
  }
}
