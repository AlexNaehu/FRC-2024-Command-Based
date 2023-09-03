package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SwerveXboxCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    
    private final XboxController controller1 = new XboxController(OIConstants.kDriverControllerPort);


    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveXboxCmd(
                swerveSubsystem,
                () -> -controller1.getLeftY(),
                () -> controller1.getLeftX(),
                () -> controller1.getRightY(),
                () -> !controller1.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); //may change

        configureButtonBindings();
    }

    private void configureButtonBindings() {
      //SwerveDrive
        new JoystickButton(controller1, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
      //Intake
      new JoystickButton(controller1, 11).whileTrue(new IntakeCmd(intakeSubsystem, true, controller1.getLeftTriggerAxis()));
      new JoystickButton(controller1, 12).whileTrue(new IntakeCmd(intakeSubsystem, false, controller1.getRightTriggerAxis()));
      //Preset Arm Movement
      //new JoystickButton(controller1, XboxController.Button.kA.value);
      //new JoystickButton(controller1, XboxController.Button.kB.value);
      //new JoystickButton(controller1, XboxController.Button.kX.value);
      //new JoystickButton(controller1, XboxController.Button.kY.value);
    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), //initial waypoint
                List.of(
                        new Translation2d(1, 0), //intermediary waypoints
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)), //final waypoint and orientation
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
