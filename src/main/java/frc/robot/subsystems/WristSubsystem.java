

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismStates.WristState;


public class WristSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final MotionMagicVoltage controller;
    private final CANcoder m_cc = new CANcoder(41);  // CANCoder with ID 19

    private WristState currentState = WristState.Home;
    private WristState requestedState = this.currentState;
    private double desiredPosition = this.currentState.getValue();

    private ShuffleboardTab tab = Shuffleboard.getTab("Mechanisms");
    private GenericEntry readyEntry = tab.add("Wrist Ready", false).getEntry();
    private GenericEntry positionEntry = tab.add("Wrist Position", 0).getEntry();
    private GenericEntry desiredPositionEntry = tab.add("Wrist Desired Position", 0).getEntry();
    private GenericEntry currentStateEntry = tab.add("Current Wrist State", "").getEntry();


    public WristSubsystem() {
        motor = new TalonFX(40);

        // Configure motor to use the Fused Cancoder as feedback sensor
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set Fused CANCoder as feedback Source
        talonFXConfigs.Feedback.FeedbackRemoteSensorID = 41;
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfigs.Feedback.SensorToMechanismRatio = 1.0;  // Ratio for the mechanism
        talonFXConfigs.Feedback.RotorToSensorRatio = 22.2222; // Ratio for the sensor and mechanism

        // Set Magnet Sensor configurations
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        magnetConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;  // Example: CounterClockwise as positive direction
        magnetConfig.MagnetOffset = -0.08;  // Adjust this value if necessary
        magnetConfig.AbsoluteSensorDiscontinuityPoint = 0.875;
        cc_cfg.withMagnetSensor(magnetConfig);  // Apply the magnet sensor configurations
        m_cc.getConfigurator().apply(cc_cfg);

        // Motion Magic PID settings.
        var slot0Configs = talonFXConfigs.Slot0;
        
        slot0Configs.kS = 0.40;
        slot0Configs.kV = 2.05;
        slot0Configs.kA = 0.05;
        slot0Configs.kG = 0.0;
        slot0Configs.kP = 75.0;
        slot0Configs.kI = 0.025;
        slot0Configs.kD = 0.00;

        // Motion Magic Settings;
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 1.50;
        motionMagicConfigs.MotionMagicAcceleration = 10;
        motionMagicConfigs.MotionMagicJerk = 250;

        // Apply configurations
        motor.getConfigurator().apply(talonFXConfigs);
        controller = new MotionMagicVoltage(0);

    }


      public void setWristBrake() {
        motor.setNeutralMode(NeutralModeValue.Brake);     
        }
    
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

          SmartDashboard.putNumber("Wrist CANCoder Absolute Position", m_cc.getAbsolutePosition().getValueAsDouble());
          SmartDashboard.putNumber("Wrist Cancoder position", m_cc.getPosition().getValueAsDouble());
          SmartDashboard.putNumber("Wrist Talon Position", motor.getPosition().getValueAsDouble()); 
          SmartDashboard.putNumber("Wrist Set Point", requestedState.getValue());
          SmartDashboard.putNumber("Wrist Stator Current", motor.getStatorCurrent().getValueAsDouble());
        }
      
        public void request(WristState state) {
          desiredPosition = state.getValue();
          requestedState = state;
        }
      
        public void setPosition(double setpoint) {
          motor.setControl(controller.withPosition(setpoint));
        }
      
        public double getPosition() {
          return motor.getPosition().getValueAsDouble();
        }
      
        public void resetPosition() {
          motor.setPosition(0);
        }
      
        public WristState getState() {
          return currentState;
        }
      
        public boolean atSetpoint() {
          return getError() <= 0.1;
        }
      
        public double getError() {
          return Math.abs(getPosition() - desiredPosition);
        }
      
        public void stop() {
          motor.stopMotor();
        }
      
      }

        //    motor.setControl(controller.withPosition(positionSetPoint));

       
        // TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          //  m_wrist.getConfigurator().refresh(appliedConfig); 

        //SmartDashboard.putNumber("Wrist Rotor to Sensor Ratio", appliedConfig.Feedback.RotorToSensorRatio);
       