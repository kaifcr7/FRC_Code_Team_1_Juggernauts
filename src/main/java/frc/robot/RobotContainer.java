
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.RobotHandler;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CANdleSubsystem;

import frc.robot.commands.ClimbStickCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.Score;
import frc.robot.commands.RotateBButtonCommand;
import frc.robot.commands.RotateXButtonCommand;
import frc.robot.Constants.RobotState;
import frc.robot.commands.AimAtTagCommandLeft;
import frc.robot.commands.AimAtTagCommandRight;

import frc.robot.commands.AutoLoadCoralCommand;
import frc.robot.commands.AutoScoreCoralLowCommand;
import frc.robot.commands.AutoAimReefLeft;

import frc.robot.commands.CANdle_Blue_Command;
import frc.robot.commands.CANdle_Yellow_Command;
import frc.robot.commands.CANdle_Solid_White_Animation;
import frc.robot.commands.CANdle_Green_Command;
import frc.robot.commands.CANdle_Red_Command;
import frc.robot.commands.CANdle_YellowFast_Command;
import frc.robot.commands.CANdle_Orange_Command;


public class RobotContainer 
{

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController UpperController = new CommandXboxController(1);

   
    
    double elevatorTolerance = 0.1;  
    double ElevatorHome = -2.0;
    double ElevatorIntake = -8.5;
    double ElevatorL1 = -16.0;
    double ElevatorL2 = -16.00;  
    double ElevatorL3 = -28.00;
    double ElevatorL4 = -79.0;  //was 81.5
    double ElevatorAlgae = -60;

   double shoulderTolerance = 0.005;               
   public boolean ShoulderLink = true; 
   public boolean previousShoulderLink = true;    

    double ShoulderIntake = 0.19;
    double ShoulderL1 = 0.1;
    double ShoulderL2 = 0.19; 
    double ShoulderL3 = ShoulderL2;
    double ShoulderL4 = 0.4958; 
    double ShoulderUp = 0.1185;
    double ShoulderAlgae = 0.1185;
    double ShoulderClimb = 0.4000;


    double WristLoadPosition = 0.75;
    double WristScoreLeft = 0.0;
    double WristScoreRight = 0.5;
    double WristScoreAlgae = 0.125;    
    double WristHome = 0.5;
    double WristTolerance = 0.001;

    double GripperPowerIn      = 0.5;   
    double GripperPowerOut      = -0.5;    
    double GripperAutoSecondsToRun = 2.0;
    double Gripper_ArmAutoSecondsToRun = 6.0;


    double strafeLeftOffset = -0.0; 
    double strafeRightOffset = 0.0; 

    private final   CANdleSubsystem       m_Candle               = new CANdleSubsystem();
    private final   ClimbSubsystem        ClimbSubsystem         = new ClimbSubsystem();
    private final   ShoulderSubsystem     ShoulderSubsystem      = new ShoulderSubsystem();   
    public  final   ElevatorSubsystem     ElevatorSubsystem      = new ElevatorSubsystem(); 
    public  final   WristSubsystem        WristSubsystem         = new WristSubsystem();
    public final    GripperSubsystem      GripperSubsystem       = new GripperSubsystem(); 
    public final    CommandSwerveDrivetrain  drivetrain          = TunerConstants.createDrivetrain();
    private final   VisionSubsystem         VisionSubsystem      = new VisionSubsystem("FrontCenter");

    public final RobotHandler robotHandler = new RobotHandler(ElevatorSubsystem, ShoulderSubsystem, WristSubsystem,
                        UpperController);
 
    private static final Command ClimbStickCommand = null;    
   
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() 
    {
        // ElevatorSubsystem.setElevatorBrake();
        // ShoulderSubsystem.setShoulderBrake();
        WristSubsystem.setWristBrake(); 
        GripperSubsystem.setGripperBrake();
    
        
      CommandScheduler.getInstance().setDefaultCommand(ClimbSubsystem, ClimbStickCommand);


      
       configureBindings();

            

        NamedCommands.registerCommand("AutoScoreCoralLowCommand", new   AutoScoreCoralLowCommand ( GripperSubsystem,
                                                                                                   GripperPowerOut,
                                                                                                   GripperAutoSecondsToRun));

        // NamedCommands.registerCommand("AutoCoralLowArmCommand", new AutoScoreCoralLowArmCommand  ( ShoulderSubsystem,
        //                                                                                                 ShoulderL1,
        //                                                                                                 shoulderTolerance,
        //                                                                                      GripperSubsystem,
        //                                                                                           GripperPowerOut,
        //                                                                                           Gripper_ArmAutoSecondsToRun));                                                                                      
      
        NamedCommands.registerCommand("AutoLoadCoral", new   AutoLoadCoralCommand ( GripperSubsystem,
                                                                                              GripperPowerIn,
                                                                                              GripperAutoSecondsToRun));

        //  NamedCommands.registerCommand("AutoScoreLevel3", new AutoScoreLevel3 ( ShoulderSubsystem,
        //                                                                                         ShoulderL3,
        //                                                                                         shoulderTolerance,
        //                                                                                         ShoulderUp,
        //                                                                          GripperSubsystem,
        //                                                                                         GripperPowerOut,                                                                                               
        //                                                                           ElevatorSubsystem,
        //                                                                                               ElevatorL3,
        //                                                                                               elevatorTolerance,
        //                                                                                               ElevatorHome));  

           NamedCommands.registerCommand("AutoAimReefLeft", new AutoAimReefLeft (VisionSubsystem, 
                                                                                            drivetrain,
                                                                                            strafeLeftOffset));                                                         
    
    
     autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
                                                                                     
                                                                                         
    }
 
    private void configureBindings() 
    {

      drivetrain.setDefaultCommand        
      (
              drivetrain.applyRequest(() ->
              drive.withVelocityX(-joystick.getLeftY() * Math.abs(joystick.getLeftY()) * (1-0.75 * joystick.getLeftTriggerAxis()) * MaxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(-joystick.getLeftX() * Math.abs(joystick.getLeftX()) * (1-0.75*joystick.getLeftTriggerAxis()) * MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(-joystick.getRightX() * Math.abs(joystick.getRightX())* (1-0.75*joystick.getLeftTriggerAxis()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
       )
      );
     

      joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

       //joystick.x().whileTrue(RotateXButtonCommand.rotate60Degrees(drivetrain));
       //joystick.b().whileTrue(RotateBButtonCommand.rotate60Degrees(drivetrain));

       joystick.button(7).onTrue(runTranslationSysId(SysIdRoutine.Direction.kForward));
       joystick.button(8).onTrue(runTranslationSysId(SysIdRoutine.Direction.kReverse));



        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(0.5))
    );

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        m_Candle.setDefaultCommand(new CANdle_Yellow_Command(m_Candle));

    //******************************************************************/
    //                      PhotonVision Aim at Reef Target
    //*****************************************************************/

   joystick.x()
        .whileTrue( new AimAtTagCommandLeft(drivetrain, VisionSubsystem, 0.165));

     joystick.x()
                         .whileTrue(new CANdle_Red_Command(m_Candle));

     joystick.b()
         .whileTrue( new AimAtTagCommandRight(drivetrain, VisionSubsystem, -0.165));
                               
        joystick.b()
                                             .whileTrue(new CANdle_Orange_Command(m_Candle));


 //******************************************************************/
 //                                State Control
 //******************************************************************/
        UpperController.start().onTrue(
            robotHandler.request(RobotState.Home));

        UpperController.rightBumper().onTrue(
            robotHandler.request(RobotState.Coral_Intake));

        UpperController.a().onTrue(
            robotHandler.request(RobotState.Coral_L1));

        UpperController.x().onTrue(
            robotHandler.request(RobotState.Coral_L2));

        UpperController.y().onTrue(
            robotHandler.request(RobotState.Coral_L3));

        UpperController.b().onTrue(
            robotHandler.request(RobotState.Coral_L4));

    

        UpperController.pov(0).onTrue( new Score(RobotState.Coral_L3, GripperSubsystem,ElevatorSubsystem,ShoulderSubsystem,WristSubsystem));
        UpperController.pov(90).onTrue( new Score(RobotState.Coral_L4, GripperSubsystem,ElevatorSubsystem,ShoulderSubsystem,WristSubsystem));
        UpperController.pov(180).onTrue( new Score(RobotState.Coral_L1, GripperSubsystem,ElevatorSubsystem,ShoulderSubsystem,WristSubsystem));
        UpperController.pov(270).onTrue( new Score(RobotState.Coral_L2, GripperSubsystem,ElevatorSubsystem,ShoulderSubsystem,WristSubsystem));

    //  UpperController.leftBumper()
    //              .onTrue(new InstantCommand (() ->new Score(robotHandler.request(RobotState.Coral_L2).schedule())));

 //******************************************************************/
 //                                Shoulder Joystick
 //******************************************************************/
//    UpperController.leftStick()
//    .onTrue(new ShoulderJoyStickCommand(ShoulderSubsystem,
//                                 () -> MathUtil.applyDeadband(UpperController.getLeftY(), 0.02) ));


//*****************************************************************/
//                        Gripper Intake
//****************************************************************/
new Trigger(() -> UpperController.getLeftTriggerAxis() >0.5)
       .whileTrue(new GripperCommand( GripperSubsystem,GripperPowerIn));

new Trigger(() -> UpperController.getLeftTriggerAxis() >0.5)
       .whileTrue(new CANdle_Solid_White_Animation(m_Candle));
                  
//*****************************************************************/
//                         Gripper Outake
//******************************************************************/

 new Trigger(() -> UpperController.getRightTriggerAxis() >0.5)
        .whileTrue( new GripperCommand( GripperSubsystem,GripperPowerOut));

       
 new Trigger(() -> UpperController.getRightTriggerAxis() >0.5)
            .whileTrue(new CANdle_Green_Command(m_Candle));

new Trigger(() -> joystick.getRightTriggerAxis() >0.5)
       .whileTrue(new GripperCommand( GripperSubsystem,GripperPowerIn));

new Trigger(() -> joystick.getRightTriggerAxis() >0.5)
        .whileTrue(new CANdle_Green_Command(m_Candle));

    //******************************************************************/
    //                        Climber Joystick
    //*****************************************************************/
    UpperController.rightStick()
    .onTrue(new ClimbStickCommand(ClimbSubsystem,
                                     () -> MathUtil.applyDeadband(-UpperController.getRightY(), 0.01) ));   
                                     
    UpperController.rightStick() 
                          .whileTrue( new CANdle_Blue_Command(m_Candle));

    // UpperController.rightStick().onTrue(
    //                       Commands.parallel(new ShoulderAngleCommand(ShoulderSubsystem, ShoulderClimb, shoulderTolerance),
    //                       new ElevatorFixedLocationCommand(ElevatorSubsystem, ElevatorHome, elevatorTolerance) ));                        


       
  }


    public Command getAutonomousCommand() 
    {
       return autoChooser.getSelected();
    }


   public ElevatorSubsystem getElevatorSubsystem()
   {
    return ElevatorSubsystem;

   }

   public ShoulderSubsystem getShoulderSubsystem()
   {
     return ShoulderSubsystem;

     }
     public WristSubsystem getWristSubsystem()
     {
        return WristSubsystem;

     }

     public GripperSubsystem getGripperSubsystem()
     {
        return GripperSubsystem;

     }


public Command runTranslationSysId(SysIdRoutine.Direction direction) {
    return drivetrain.sysIdQuasistatic(direction); // or drivetrain.sysIdDynamic(direction);
}

public Command runRotationSysId(SysIdRoutine.Direction direction) {
    return drivetrain.sysIdDynamic(direction);
}}
