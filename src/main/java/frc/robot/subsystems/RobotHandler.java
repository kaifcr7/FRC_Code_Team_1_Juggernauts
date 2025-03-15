// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotState;
import frc.robot.Constants.MechanismStates.ShoulderState;
import frc.robot.Constants.MechanismStates.ElevatorState;
import frc.robot.Constants.MechanismStates.WristState;

public class RobotHandler extends SubsystemBase {

  private final ElevatorSubsystem elevatorSubsystem;
  private final ShoulderSubsystem shoulderSubsystem;
  private final WristSubsystem wristSubsystem;
  private final CommandXboxController xboxController_1;

  private RobotState currentState = RobotState.Home;
  private RobotState requestedState = this.currentState;

  private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
  private GenericEntry readyEntry = tab.add("Arm Ready", false).getEntry();
  private GenericEntry requestedStateEntry = tab.add("Requested Arm State", "").getEntry();
  private GenericEntry currentStateEntry = tab.add("Current Arm State", "").getEntry();

  public RobotHandler(ElevatorSubsystem elevatorSubsystem, ShoulderSubsystem shoulderSubsystem,
      WristSubsystem wristSubsystem, CommandXboxController xboxController_1) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.shoulderSubsystem = shoulderSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.xboxController_1 = xboxController_1;
  }

  @Override
  public void periodic() {
    // if (elevatorSubsystem.getState() == requestedState.getElevatorState()
    //     && shoulderSubsystem.getState() == requestedState.getShoulderState()
    //     && wristSubsystem.getState() == requestedState.getWristState()) {
    //   currentState = requestedState;
    //   readyEntry.setBoolean(true);
    //   xboxController_1.setRumble(RumbleType.kBothRumble, 0.5);
    // } else {
    //   readyEntry.setBoolean(false);
    // }
    //  System.out.println(requestedState);
    // requestedStateEntry.setString(requestedState.toString());
    // currentStateEntry.setString(currentState.toString());
  }

  public Command request(RobotState state) {
    requestedState = state;
    ElevatorState requestedElevatorState = state.getElevatorState();
    ShoulderState requestedShoulderState = state.getShoulderState();
    WristState requestedWristState = state.getWristState();


    return new InstantCommand(() -> wristSubsystem.request(requestedWristState),
          wristSubsystem)
          .alongWith(new InstantCommand(
              () -> shoulderSubsystem.request(requestedShoulderState),
              shoulderSubsystem))
          .alongWith(new InstantCommand(
              () -> elevatorSubsystem.request(requestedElevatorState),
              elevatorSubsystem))
          .alongWith(new InstantCommand(() -> requestedState = state));
            }

  public RobotState getState() {
    return requestedState;
  }

  
    // int counter = 0;

    // // Count the number of mechanisms moving up
    // if (requestedElevatorState.getValue() - currentElevatorState.getValue() > 0) {
    //   counter++;
    // }

    // if (requestedShoulderState.getValue() - currentShoulderState.getValue() > 0) {
    //   counter++;
    // }

    // if (requestedWristState.getValue() - currentWristState.getValue() > 0) {
    //   counter++;
    // }

    // // If the number of mechanisms moving up is greater or equal to 2, treat the arm
    // // motion as moving up. Otherwise, treat it as moving down
    // if (counter >= 2) {
    //   // Moving up

    //   return new InstantCommand(() -> elevatorSubsystem.request(requestedElevatorState),
    //       elevatorSubsystem)
    //       .andThen(Commands.waitSeconds(0.1))
    //       .andThen(new InstantCommand(
    //           () -> ShoulderSubsystem.request(requestedShoulderState),
    //           shoulderSubsystem))
    //       .andThen(Commands.waitSeconds(0.1))
    //       .andThen(new InstantCommand(
    //           () -> wristSubsystem.request(requestedWristState),
    //           wristSubsystem));

    // } else {
    //   // Moving down

    //   return new InstantCommand(() -> wristSubsystem.request(requestedWristState),
    //       wristSubsystem)
    //       .andThen(Commands.waitSeconds(0.1))
    //       .andThen(new InstantCommand(
    //           () -> ShoulderSubsystem.request(requestedShoulderState),
    //           shoulderSubsystem))
    //       .andThen(Commands.waitSeconds(0.1))
    //       .andThen(new InstantCommand(
    //           () -> elevatorSubsystem.request(requestedElevatorState),
    //           elevatorSubsystem));
    // }


}
