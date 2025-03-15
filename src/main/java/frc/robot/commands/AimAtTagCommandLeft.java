// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AimAtTagCommandLeft extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final VisionSubsystem vision;
    private double strafeOffsetMeters;

    public AimAtTagCommandLeft(CommandSwerveDrivetrain swerve, VisionSubsystem vision, double initialOffset) {
        this.swerve = swerve;
        this.vision = vision;
        this.strafeOffsetMeters = initialOffset;
        
        addRequirements(vision, swerve); // Ensure only one command uses these subsystems at a time
    }

    public void setOffset(double newOffset) {
        this.strafeOffsetMeters = newOffset;
    }

    @Override
    public void execute() {
        if (vision.hasValidTarget()) {
            double strafeSpeed = vision.getStrafeSpeedWithOffset(strafeOffsetMeters);
                      
            swerve.drive(0, strafeSpeed, 0);

        } else {
            swerve.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return vision.hasValidTarget() 
                    && Math.abs(vision.getStrafeSpeedWithOffset(strafeOffsetMeters)) < 0.02;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
