

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.Constants.MechanismStates.ShoulderState;

public class ShoulderSubsystem extends SubsystemBase {

    // PID and control constants
    private double kP = 35.0;  // Proportional gain  was 0.5
    private double kI = 0.2;    // Integral gain
    private double kD = 0.0;  // Derivative gain was 0.02
    private double kS = 0.0; // Static friction compensation
    private double kV = 0.0; // Velocity term for overcoming friction

    private double positionSetPoint = 0.1185;
    private boolean ShoulderLink;
   

    private final TalonFX m_shoulder = new TalonFX(18);  // TalonFX motor
    private final CANcoder m_cc = new CANcoder(19);  // CANCoder with ID 19

    public TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    public CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    private final PositionVoltage positionControl = new PositionVoltage(0);

    private ShoulderState currentState = ShoulderState.Home;
    private ShoulderState requestedState = this.currentState;
    private double desiredPosition = this.currentState.getValue();

    private ShuffleboardTab tab = Shuffleboard.getTab("Mechanisms");
    private GenericEntry readyEntry = tab.add("Elbow Ready", false).getEntry();
    private GenericEntry positionEntry = tab.add("Elbow Position", 0).getEntry();
    private GenericEntry desiredPositionEntry = tab.add("Elbow Desired Position", 0).getEntry();
    private GenericEntry currentStateEntry = tab.add("Current Elbow State", "").getEntry();

    public ShoulderSubsystem() {
        // Configure the CANCoder with ID 19
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();

        // Set Magnet Sensor configurations
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;  // Example: CounterClockwise as positive direction
        magnetConfig.MagnetOffset = 0.56;  // Adjust this value if necessary
        cc_cfg.withMagnetSensor(magnetConfig);  // Apply the magnet sensor configurations

        // Apply the CANCoder configuration
        m_cc.getConfigurator().apply(cc_cfg);

        // Configure the TalonFX motor for position control (PID)
        fx_cfg.Feedback.FeedbackRemoteSensorID = m_cc.getDeviceID();  // Use CANCoder for feedback
        fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        //fx_cfg.Feedback.SensorToMechanismRatio = 1.0;  // Ratio for the mechanism
        fx_cfg.Feedback.RotorToSensorRatio = 81.26984; // Ratio for the sensor and mechanism
        fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_shoulder.getConfigurator().apply(fx_cfg);
        Timer.delay(0.1);

        // Set neutral mode to Brake
        m_shoulder.setNeutralMode(NeutralModeValue.Brake);

        // Set motor voltage and current limits
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Voltage.PeakForwardVoltage = 5;
        configs.Voltage.PeakReverseVoltage = -5;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;                    
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;               // was 40
        m_shoulder.getConfigurator().apply(configs);

        // Apply current limit configuration
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 40;                               //was 40
        limitConfigs.StatorCurrentLimitEnable = true;
        m_shoulder.getConfigurator().apply(limitConfigs);

       
        m_shoulder.getConfigurator().apply(new TalonFXConfiguration());
        m_shoulder.getConfigurator().apply(fx_cfg);

        // Configure PID constants for TalonFX
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        m_shoulder.getConfigurator().apply(slot0Configs);


    }

    // // Set shoulder position (PID controlled)
    // public void setShoulderPosition(double positionSetPoint) 
    // {
    //     this.positionSetPoint = positionSetPoint;

       
    //     m_shoulder.setControl(positionControl.withPosition(positionSetPoint));
    // }

    // // Set shoulder motor speed directly (open loop control)
    // public void setShoulderSpeed(double percentOutput) {
    //     m_shoulder.set(percentOutput);  
    // }

    // // Get the shoulder position from the CANCoder
    // public double getPosition() {
    //     return m_cc.getAbsolutePosition().getValueAsDouble();
    // }

    // // Set shoulder to coast mode (no braking)
    // public void setShoulderCoast() {    
    //     m_shoulder.setNeutralMode(NeutralModeValue.Coast);        
    // }

    // // Set shoulder to brake mode
    // public void setShoulderBrake() {
    //     m_shoulder.setNeutralMode(NeutralModeValue.Brake);     
    // }


    // public void setShoulderLink(boolean y)
    // {
    //     ShoulderLink = y;
    // }

    // @Override
    // public void periodic() {
       
       
    //     // TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
    //       //  m_shoulder.getConfigurator().refresh(appliedConfig); 

    //     //SmartDashboard.putNumber("Shoulder Rotor to Sensor Ratio", appliedConfig.Feedback.RotorToSensorRatio);
    //     SmartDashboard.putNumber("Shoulder CANCoder Absolute Position", m_cc.getAbsolutePosition().getValueAsDouble());
    //     SmartDashboard.putNumber("Cancoder position", m_cc.getPosition().getValueAsDouble());
    //     SmartDashboard.putNumber("Talon Position", m_shoulder.getPosition().getValueAsDouble()); 
    //     SmartDashboard.putNumber("Shoulder Set Point", positionSetPoint);
    //     SmartDashboard.putNumber("Shoulder Stator Current", m_shoulder.getStatorCurrent().getValueAsDouble());
    //     SmartDashboard.putBoolean("ShoulderLink Boolean = ", ShoulderLink);
      
    // }


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

  public void request(ShoulderState state) {
    desiredPosition = state.getValue();
    requestedState = state;
  }

  public void setPosition(double setpoint) {
    m_shoulder.setControl(positionControl.withPosition(setpoint));
  }

  public double getPosition() {
    return m_shoulder.getPosition().getValueAsDouble();
  }

  public void resetPosition() {
    m_shoulder.setPosition(0);
  }

  public ShoulderState getState() {
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
    m_shoulder.stopMotor();
  }
}

       