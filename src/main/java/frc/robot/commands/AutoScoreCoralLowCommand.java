// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GripperSubsystem;


public class AutoScoreCoralLowCommand  extends Command 
{
  
  private double SecondsToRun;  
  
  private double ExecuteTimeStamp;
  private double isFinishedTimeStamp  = 0.0;
  private double GripperPercentOut; 
  private double lastTimeStamp = 0.0;
  private double CurrentTimeStamp = 0.0;
  private final GripperSubsystem  GripperSubsystem;

  public AutoScoreCoralLowCommand( GripperSubsystem GripperSubsystem,
                                                          double GripperPercentOutput,
                                                          double SecondstoRun)

  {      
   
    this.GripperSubsystem = GripperSubsystem;
    this.GripperPercentOut = GripperPercentOutput;
    this.SecondsToRun = SecondstoRun;   
    addRequirements(GripperSubsystem);
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
      
   
        GripperSubsystem.setGripperSpeed(GripperPercentOut);
        SmartDashboard.putNumber("AutoCoral sec to run", SecondsToRun);
       
      
  }

  
  @Override
  public void end(boolean interrupted) 
  {
    isFinishedTimeStamp = Timer.getFPGATimestamp();
    
    GripperSubsystem.setGripperSpeed(0); 
   
  }

  @Override
  public boolean isFinished()
  {
    if((Timer.getFPGATimestamp() -  ExecuteTimeStamp) > SecondsToRun) 
    {
      return true;
    } 
    return false;
  }
}
