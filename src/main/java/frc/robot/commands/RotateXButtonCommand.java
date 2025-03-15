

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotateXButtonCommand extends Command{
    public static Command rotate60Degrees(CommandSwerveDrivetrain drivetrain) {
        return Commands.sequence(
            // Set target yaw once
            Commands.runOnce(() -> drivetrain.setTargetYaw(60), drivetrain),
            
            // Continuously rotate until within 2 degrees
            Commands.run(() -> {
                double error = drivetrain.getYawError();
                double rotationSpeed = error * 0.07; // Simple P-control
               

                if (Math.abs(error) > 2) {
                    drivetrain.drive(0, 0, rotationSpeed);
                } else {
                    drivetrain.stop();
                }
            }, drivetrain).until(() -> Math.abs(drivetrain.getYawError()) < 2.0),

            // Stop and reset yaw after finishing
            Commands.runOnce(() -> {
                drivetrain.stop();
                drivetrain.clearTargetYaw();
            }, drivetrain)
        );
    }
}
