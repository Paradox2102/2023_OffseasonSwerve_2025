// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {
  /** Creates a new ManualElevatorCommand. */
  ElevatorSubsystem m_subsystem;
  DoubleSupplier m_up;
  public ManualElevatorCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier up) {
    m_subsystem = elevatorSubsystem;
    m_up = up;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double up = m_up.getAsDouble();
    if (up == 0) {
      m_subsystem.manualControl(true, false);
    } else {
      m_subsystem.manualControl(up < 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.manualControl(true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
