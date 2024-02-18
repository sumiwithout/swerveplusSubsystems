// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FeedLauncher;
import frc.robot.commands.Retract;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.utils.GamepadUtils;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
private final Retract retract = new Retract(m_intake);
private final FeedLauncher feedLauncher = new FeedLauncher(m_intake, m_launcher);
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -GamepadUtils.squareInput(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -GamepadUtils.squareInput(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -GamepadUtils.squareInput(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true,
                    false),
            m_robotDrive));

    // set the arm subsystem to run the "runAutomatic" function continuously when no other command is running
    m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

    // set the intake to stop (0 power) when no other command is running
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

    // configure the launcher to stop when no other command is running
    m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // button to put swerve modules in an "x" configuration to hold position
    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // set up arm preset positions
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));
    new Trigger(
            () ->
                m_driverController.getLeftTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));

    // intake controls (run while button is held down, run retract command once when the button is released)
    new Trigger(
            () ->
                m_driverController.getRightTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), m_intake))
        .onFalse(retract);

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(-1.0)));

    // launcher controls (button to pre-spin the launcher and button to launch)
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_launcher.runLauncher(), m_launcher));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(feedLauncher);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         config);

    // var thetaController =
    //     new ProfiledPIDController(
    //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand =
    //     new SwerveControllerCommand(
    //         exampleTrajectory,
    //         m_robotDrive::getPose, // Functional interface to feed supplier
    //         DriveConstants.kDriveKinematics,

    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         m_robotDrive::setModuleStates,
    //         m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));


    AutoBuilder.configureHolonomic(
      m_robotDrive::getPose, // Robot pose supplier
      m_robotDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      m_robotDrive::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      m_robotDrive::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
              4.5, // Max module speed, in m/s
              0.4, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          Alliance alliance = DriverStation.getAlliance();
          if (alliance != null) {
            boolean ret = (alliance == DriverStation.Alliance.Red);
              return ret;
          }
          return false;
      },
      m_robotDrive // Reference to this subsystem to set requirements
);
PathPlannerPath path = PathPlannerPath.fromPathFile("follow");
    

return AutoBuilder.followPath(path);
  }
}
