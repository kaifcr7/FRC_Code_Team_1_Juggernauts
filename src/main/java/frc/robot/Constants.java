
package frc.robot;

import frc.robot.Constants.MechanismStates.ElevatorState;
import frc.robot.Constants.MechanismStates.ShoulderState;
import frc.robot.Constants.MechanismStates.WristState;

public class Constants 

{

    public final class VisionConstants {
        public static final double kP = 0.05; // Adjust for better tuning
        public static final double kI = 0.00;
        public static final double kD = 0.000;
        public static final double kS = 0.1;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double angleTolerance = 1.0; // Degrees
        //public static double k = 0.0394;
    }


public enum RobotState {
    Home(ElevatorState.Home, ShoulderState.Home, WristState.Home),
    Coral_Intake(ElevatorState.CI, ShoulderState.CI, WristState.CI),
    Coral_L1(ElevatorState.CL1, ShoulderState.CL1, WristState.CL1),
    Coral_L2(ElevatorState.CL2, ShoulderState.CL2, WristState.CL2),
    Coral_L3(ElevatorState.CL3, ShoulderState.CL3, WristState.CL3),
    Coral_L4(ElevatorState.CL4, ShoulderState.CL4, WristState.CL4),
    Algae_Intake(ElevatorState.AI, ShoulderState.AI, WristState.AI),
    Algae_L2(ElevatorState.AL2, ShoulderState.AL2, WristState.AL2),
    Algae_L3(ElevatorState.AL3, ShoulderState.AL3, WristState.AL3),
    Algae_Processor(ElevatorState.AP, ShoulderState.AP, WristState.AP),
    Algae_Net(ElevatorState.AN, ShoulderState.AN, WristState.AN),
    Climb(ElevatorState.Climb, ShoulderState.Climb, WristState.Climb);

    private final ElevatorState elevatorState;
    private final ShoulderState shoulderState;
    private final WristState wristState;

    RobotState(ElevatorState elevatorState, ShoulderState shoulderState, WristState wristState) {
      this.elevatorState = elevatorState;
      this.shoulderState = shoulderState;
      this.wristState = wristState;
    }

    public ElevatorState getElevatorState() {
      return this.elevatorState;
    }

    public ShoulderState getShoulderState() {
      return this.shoulderState;
    }

    public WristState getWristState() {
      return this.wristState;
    }
  }

  public static class MechanismStates {
    public enum ShoulderState {
      Home(0.1185),
      CI(0.0715),
      CL1(0.3149),
      CL2(0.22),
      CL3(0.22),
      CL4(0.22),
      AI(0.1185),
      AL2(0.1185),
      AL3(0.1185),
      AP(0.1185),
      AN(0.1185),
      Climb(0.4000),
      SCOREL2(0.30),
      SCOREL3(0.30),
      SCOREL4(0.30);

      private final double value;

      ShoulderState(double value) {
        this.value = value;
      }

      public double getValue() {
        return value;
      }
    }

    public enum WristState {
      Home(0.5),
      CI(0.75),
      CL1(0.25),
      CL2(0.5),
      CL3(0.5),
      CL4(0.5),
      AI(0.5),
      AL2(0.5),
      AL3(0.5),
      AP(0.5),
      AN(0.5),
      Climb(0.5);


      private final double value;

      WristState(double value) {
        this.value = value;
      }

      public double getValue() {
        return value;
      }
    }

    public enum ElevatorState {
      Home(-2.0),
      CI(-8.5),
      CL1(-11.0),
      CL2(-14.0),
      CL3(-27.0),
      CL4(-55.0),
      AI(0),
      AL2(0),
      AL3(0),
      AP(0),
      AN(0),
      Climb(0),
      SCOREL2(-8.0),
      SCOREL3(-12.0),
      SCOREL4(-41.0);

      private final double value;

      ElevatorState(double value) {
        this.value = value;
      }

      public double getValue() {
        return value;
      }
    }
  }
    


}

