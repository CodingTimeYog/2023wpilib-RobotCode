// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlatformDockPidCommand_Y extends PIDCommand {
  private final static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final static AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  private static double last_pitch_x = 0.00;
  public PlatformDockPidCommand_Y() {
    super(
        // The controller that the command will use
        new PIDController(Constants.kPPitch, Constants.kIPitch, Constants.kDPitch),  () -> smoothpitch(m_navx.getPitch()),
        // This is the setpoint(can be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // This is what the robot will do
          m_drivetrainSubsystem.drive_pid_y(output);
        });
        addRequirements(m_drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }
  private static double smoothpitch(float pitch) {
    double calculatedpitch=pitch;
    SmartDashboard.putNumber("read pitch", pitch);
    if (Math.abs(pitch) < 2.5)
    {
      calculatedpitch=0.00000001;
      last_pitch_x=0.00000001;

    }
    else if(Math.abs(pitch) < 40)
    {
      calculatedpitch=pitch;
      last_pitch_x=calculatedpitch;

    }
    else
    {
      //calcuatedroll=20*roll/Math.abs(roll);
      calculatedpitch=last_pitch_x;
      //calculatedroll=0.001*roll/Math.abs(roll);
    }
    SmartDashboard.putNumber("set pitch", calculatedpitch);
    return calculatedpitch;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
