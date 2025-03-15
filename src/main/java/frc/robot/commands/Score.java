package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;


import frc.robot.Constants.RobotState;

import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

import frc.robot.Constants.MechanismStates.ShoulderState;
import frc.robot.Constants.MechanismStates.ElevatorState;
import frc.robot.Constants.MechanismStates.WristState;

import frc.robot.commands.GripperCommand;

public class Score extends SequentialCommandGroup {

private final GripperSubsystem gripperSubsystem;
private final ElevatorSubsystem elevatorSubsystem;
private final ShoulderSubsystem shoulderSubsystem;
private final WristSubsystem wristSubsystem;
private final RobotState globalRobotState;
// private final Timer timer = new Timer();
private boolean actionCompleted = false;


  public Score(RobotState globalRobotState, GripperSubsystem gripperSubsystem, ElevatorSubsystem elevatorSubsystem, ShoulderSubsystem shoulderSubsystem, WristSubsystem wristSubsystem) {
    this.globalRobotState = globalRobotState;
    this.gripperSubsystem = gripperSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.shoulderSubsystem = shoulderSubsystem;
    this.wristSubsystem = wristSubsystem;
    addRequirements(gripperSubsystem, elevatorSubsystem, shoulderSubsystem, wristSubsystem);

    actionCompleted = false;

    switch (globalRobotState) {
      case Coral_L1:
        addCommands(
          new InstantCommand(()-> gripperSubsystem.setGripperSpeed(0.3)),
          new WaitCommand(0.2),
          new InstantCommand(()-> gripperSubsystem.setGripperSpeed(0))
        );
        break;

        case Coral_L2:
        addCommands(
          new InstantCommand(() -> shoulderSubsystem.request(ShoulderState.SCOREL2)),
          new WaitCommand(0.3),
          new InstantCommand(()-> gripperSubsystem.setGripperSpeed(0.3)),
          new WaitCommand(0.1),
          new InstantCommand(()-> gripperSubsystem.setGripperSpeed(0)),
          new InstantCommand(() -> elevatorSubsystem.request(ElevatorState.SCOREL2))
        );
        break;

      case Coral_L3:
        addCommands(
          new InstantCommand(() -> shoulderSubsystem.request(ShoulderState.SCOREL3)),
          new WaitCommand(0.3),
          new InstantCommand(()-> gripperSubsystem.setGripperSpeed(0.3)),
          new WaitCommand(0.1),
          new InstantCommand(()-> gripperSubsystem.setGripperSpeed(0)),
          new InstantCommand(() -> elevatorSubsystem.request(ElevatorState.SCOREL3))
        );
        break;

      case Coral_L4:
      addCommands(
        new InstantCommand(() -> shoulderSubsystem.request(ShoulderState.SCOREL4)),
        new WaitCommand(0.5),
        new InstantCommand(() -> elevatorSubsystem.request(ElevatorState.SCOREL4))
      );
        break;

      default:
        actionCompleted = true;

    }

  }

}