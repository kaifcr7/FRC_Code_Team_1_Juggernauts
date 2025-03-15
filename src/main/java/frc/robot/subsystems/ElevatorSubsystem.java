
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismStates.ElevatorState;


public class ElevatorSubsystem extends SubsystemBase {

    private final TalonFX m_elevator = new TalonFX(15, "ChassisCAN");  // Main elevator motor
    private final TalonFX m_elevator2 = new TalonFX(16, "ChassisCAN");  // Follower motor
    private final DigitalInput DIO = new DigitalInput(0);  // Limit switch
    private final PositionVoltage positionControl = new PositionVoltage(0);
    private        AnalogInput  elevatorHomeSensor = new AnalogInput(0);
    

// PID constants
    private double kP = 2.5;  // Proportional gain
    private double kI = 0.0;  // Integral gain
    private double kD = 0.025;  // Derivative gain
    private double kS = 0.0;  // Static friction compensation
    private double kV = 0.0;  
    private double kG = -0.035;// Velocity term for overcoming friction
    private double homePosition = -6.0;    //This is the elevator home position
    private double homeThreshold = 0.15;  //Analog Voltage threshold to reset home positoin
    private boolean hasResetHome = false;
    private double positionSetPoint = -1.0;
    
    private ElevatorState currentState = ElevatorState.Home;
    private ElevatorState requestedState = this.currentState;
    private double desiredPosition = this.currentState.getValue();

    private ShuffleboardTab tab = Shuffleboard.getTab("Mechanisms");
    private GenericEntry readyEntry = tab.add("Elevator Ready", false).getEntry();
    private GenericEntry positionEntry = tab.add("Elevator Position", 0).getEntry();
    private GenericEntry desiredPositionEntry = tab.add("Elevator Desired Position", 0).getEntry();
    private GenericEntry currentStateEntry = tab.add("Current Elevator State", "").getEntry();


    public ElevatorSubsystem() 
    {
        // Set neutral mode for the elevator motors
        m_elevator.setNeutralMode(NeutralModeValue.Brake);
        m_elevator2.setNeutralMode(NeutralModeValue.Brake);
        m_elevator.setPosition(0);   

        var configs = new TalonFXConfiguration();
        configs.Voltage.PeakForwardVoltage = 11;
        configs.Voltage.PeakReverseVoltage = -11;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        m_elevator.getConfigurator().apply(configs);
        m_elevator2.getConfigurator().apply(configs);

        var talonFXConfigurator = m_elevator.getConfigurator();        
        var limitConfigs = new CurrentLimitsConfigs();
          limitConfigs.StatorCurrentLimit = 60;
          limitConfigs.StatorCurrentLimitEnable = true;
          talonFXConfigurator.apply(limitConfigs);

       var talonFXConfigurator2 = m_elevator2.getConfigurator();
       var limitConfigs2 = new CurrentLimitsConfigs();
           limitConfigs2.StatorCurrentLimit = 60;
           limitConfigs2.StatorCurrentLimitEnable = true;
           talonFXConfigurator2.apply(limitConfigs2);

        // Configure the TalonFX for position control (PID)
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        slot0Configs.kG = kG;
        m_elevator.getConfigurator().apply(slot0Configs);
        m_elevator2.getConfigurator().apply(slot0Configs);


         m_elevator2.setControl(new com.ctre.phoenix6.controls.Follower(15, true)); // Follower motor
    }

//    public void setElevatorPosition(double positionSetPoint) 
//     {
//         this.positionSetPoint = positionSetPoint;
//         m_elevator.setControl(positionControl.withPosition(positionSetPoint)); 
      
//     }

//        // Get the shoulder position from the CANCoder
//        public double getPosition() {
//         return m_elevator.getPosition().getValueAsDouble();
//     }

//     public void setElevatorBrake() {
//         m_elevator.setNeutralMode(NeutralModeValue.Brake);
//         m_elevator2.setNeutralMode(NeutralModeValue.Brake);
//     }

//     public void setElevatorCoast() {
//         m_elevator.setNeutralMode(NeutralModeValue.Coast);
//         m_elevator2.setNeutralMode(NeutralModeValue.Coast);
//     }

//     public void holdPosition() 
//     {
//         // Reapply the last known position to hold
//       m_elevator.setControl(positionControl.withPosition(positionSetPoint));
//     }

//     @Override
//     public void periodic() 
//     {
        
//      // Continuously hold position
//        m_elevator.setControl(positionControl.withPosition(positionSetPoint));
        
//         SmartDashboard.putNumber("Elevator Position", m_elevator.getPosition().getValueAsDouble());       
//         SmartDashboard.putNumber("Elevator Setpoint", positionSetPoint);
//         double statorCurrent = m_elevator.getStatorCurrent().getValueAsDouble();   
//         SmartDashboard.putNumber("Elevator Stator Current", statorCurrent);   
              
//     }

   
    @Override
    public void periodic() {
      setPosition(desiredPosition);
  
      if (atSetpoint()) {
        currentState = requestedState;
      }
  
      readyEntry.setBoolean(currentState == requestedState);
      positionEntry.setDouble(getPosition());
      desiredPositionEntry.setDouble(desiredPosition);
      currentStateEntry.setString(currentState.toString());
    }
  
    public void request(ElevatorState state) {
      desiredPosition = state.getValue();
      requestedState = state;
    }
  
    public void setPosition(double setpoint) {
      m_elevator.setControl(positionControl.withPosition(setpoint));
    }
  
    public double getPosition() {
      return m_elevator.getPosition().getValueAsDouble();
    }
  
    public void resetPosition() {
        m_elevator.setPosition(0);
    }
  
    public ElevatorState getState() {
      return currentState;
    }
  
    // Move this arbitrary value into constants
    public boolean atSetpoint() {
      return getError() <= 0.1;
    }
  
    public double getError() {
      return Math.abs(getPosition() - desiredPosition);
    }
  
    public void stop() {
        m_elevator.stopMotor();
    }


}

