// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveOneDirection extends CommandBase {
  /** Creates a new DriveOneDirection. */
  final DrivetrainSubsystem m_drivetrainSubsystem  = new DrivetrainSubsystem();
  public DriveOneDirection() {
    
      addRequirements(m_drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(1, 0, 0));
    SmartDashboard.putNumber("Front Left Module Velocity", m_drivetrainSubsystem.getFrontLeftVelocity());
    SmartDashboard.putNumber("Front Right Module Velocity", m_drivetrainSubsystem.getFrontRightVelocity());
    SmartDashboard.putNumber("Back Left Module Velocity", m_drivetrainSubsystem.getBackLeftVelocity());
    SmartDashboard.putNumber("Back Right Module Velocity", m_drivetrainSubsystem.getBackRightVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
