// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase 
{
    private final PhotonCamera camera;
    private PhotonTrackedTarget bestTarget;
    private final PIDController strafePID;
    private final SimpleMotorFeedforward strafeFeedforward;

      
    public VisionSubsystem(String FrontCenter) {
        camera = new PhotonCamera(FrontCenter);

        strafePID = new PIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD);
        strafePID.setTolerance(VisionConstants.angleTolerance);

        strafeFeedforward = new SimpleMotorFeedforward(VisionConstants.kS, VisionConstants.kV, VisionConstants.kA);

    }

    public boolean hasValidTarget() {
        return bestTarget != null;
    }


    public double getStrafeSpeedWithOffset(double offsetMeters) { 
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            bestTarget = result.getBestTarget();
    
            double yawError = bestTarget.getYaw(); // Use angle instead of X position
        double pidOutput = strafePID.calculate(yawError, offsetMeters);
        double desiredVelocity = strafePID.getVelocityError();
        double feedforwardOutput = strafeFeedforward.calculate(desiredVelocity);
        double strafeSpeed = pidOutput + feedforwardOutput;

        if (Math.abs(strafeSpeed) < 0.1) {
            strafeSpeed = Math.signum(strafeSpeed) * 0.1;
        }

        // Debugging prints
        System.out.println("Yaw Error: " + yawError);
        System.out.println("PID Output: " + pidOutput);
        System.out.println("Desired Velocity: " + desiredVelocity);
        System.out.println("Feedforward Output: " + feedforwardOutput);
        System.out.println("Final Strafe Speed: " + strafeSpeed);

        return strafeSpeed;
    } else {
        System.out.println("No target detected!");
    }
    return 0.0;
}

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        bestTarget = result.hasTargets() ? result.getBestTarget() : null;
        
       
    }
}

