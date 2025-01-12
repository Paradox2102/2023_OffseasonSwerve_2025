// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeReefscape extends Command {
  IntakeSubsystem m_intakeSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  WristSubsystem m_wristSubsystem;
  /** Creates a new IntakeReefscape. */
  public IntakeReefscape(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem, m_elevatorSubsystem, m_wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double wrist = Constants.ConeConstants.k_groundWristDegrees;
    double elevator = Constants.ConeConstants.k_groundElevatorInches;
    m_wristSubsystem.setAngleDegrees(wrist);
    m_elevatorSubsystem.setExtentInches(elevator);
    m_intakeSubsystem.intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stop();
    double wrist = Constants.k_neutralWristDegrees;
    double elevator = Constants.k_neutralElevatorInches;
    m_wristSubsystem.setAngleDegrees(wrist);
    m_elevatorSubsystem.setExtentInches(elevator);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
