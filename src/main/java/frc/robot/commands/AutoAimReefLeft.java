// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAimReefLeft extends Command 
{

    double AutoSecondsToRun;  
    double ExecuteTimeStamp = 0.0;
    double CurrentTimeStamp = 0.0;
    private double strafeOffsetMeters;
 
   private final VisionSubsystem vision;
   private final CommandSwerveDrivetrain swerve;

  public AutoAimReefLeft(VisionSubsystem vision,
                            CommandSwerveDrivetrain swerve, double initialOffset)

  {      
    this.vision = vision;
    this.swerve = swerve;  
    this.strafeOffsetMeters = initialOffset;
    addRequirements(vision, swerve);
  }

public void setOffset(double newOffset){
  this.strafeOffsetMeters = newOffset;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    ExecuteTimeStamp = Timer.getFPGATimestamp();
    
  }

  @Override
  public void execute() 
      {
     
        if (vision.hasValidTarget()) 
        {
          double strafeSpeed = vision.getStrafeSpeedWithOffset(strafeOffsetMeters);
        
          swerve.drive(0, strafeSpeed, 0);
        } else 
      {
          swerve.stop();
      }

    }

      @Override
      public boolean isFinished() 
      
      {
          
            
        
        return vision.hasValidTarget() && Math.abs(vision.getStrafeSpeedWithOffset(0)) < 0.02;
      }

      @Override
      public void end(boolean interrupted) {
          swerve.stop();
      }
  }

