// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
// import static frc.robot.Constants.ElevatorConstants.ELEVATOR_MAX_ROTATION;
// import static frc.robot.Constants.ElevatorConstants.ELEVATOR_MIN_ROTATION;

import java.util.function.Function;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Discharge;
import frc.robot.commands.Eject;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.Intake;
// import frc.robot.commands.Launch;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.LowerIntake;
import frc.robot.commands.RaiseIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANElevatorSubsystem;
import frc.robot.subsystems.CANIntakeArmSubsystem;
import frc.robot.subsystems.CANShooterSubsystem;
import frc.robot.subsystems.CANIntakeSubsystem;
import frc.robot.subsystems.CANSwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.*;

public class RobotContainer {
    // Changed 1.0 to 0.3
    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipController = new CommandXboxController(1);

    public final CANSwerveSubsystem drivetrain = TunerConstants.createDrivetrain();

    public final CANShooterSubsystem shooter = new CANShooterSubsystem();
    public final CANIntakeSubsystem intake = new CANIntakeSubsystem();

    public final CANElevatorSubsystem elevator = new CANElevatorSubsystem();

    public final CANIntakeArmSubsystem intakeArm = new CANIntakeArmSubsystem();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final VisionSubsystem vision = new VisionSubsystem(this::consumePoseEstimate);

    public RobotContainer() {
        NamedCommands.registerCommand("Launch Sequence", new LaunchSequence(shooter).asProxy().withTimeout(6));
        NamedCommands.registerCommand("Intake Down", new LowerIntake(intakeArm).asProxy().withTimeout(0.5));
        NamedCommands.registerCommand("Intake Up", new RaiseIntake(intakeArm).asProxy().withTimeout(0.5));
        NamedCommands.registerCommand("Run Intake", new Intake(intake).asProxy().withTimeout(99));
        NamedCommands.registerCommand("Elevator Down", new ElevatorDown(elevator).asProxy().withTimeout(6));
        NamedCommands.registerCommand("Elevator Up", new ElevatorUp(elevator).asProxy().withTimeout(6));

        // autoChooser = AutoBuilder.buildAutoChooser();
        // Manually Add Autos
        buildAutoAndAddToChooser("Center Backup-Shoot");
        buildAutoAndAddToChooser("Left Trench-Intake-Trench-Shoot");
        buildAutoAndAddToChooser("Right Trench-Intake-Trench-Shoot");
        // buildAutoAndAddToChooser("Center Backup-Shoot-Climb");
        // buildAutoAndAddToChooser("Left-Backup-Shoot-Climb");
        // buildAutoAndAddToChooser("Right-Backup-Shoot-Climb");
        autoChooser.setDefaultOption("None", null);

        CameraServer.startAutomaticCapture("Elevator", 0);
        CameraServer.startAutomaticCapture("Hopper", 1);

        SmartDashboard.putNumber("Ideal Shoot Distance", 3.0);
        SmartDashboard.putNumber("Shoot Distance Epsilon", 0.5);
        SmartDashboard.putBoolean("At Shoot Distance", false);

        SmartDashboard.putNumber("Shooter voltage", FuelConstants.SHOOTER_VOLTAGE);
        SmartDashboard.putNumber("Intake voltage", FuelConstants.INTAKE_VOLTAGE);
        SmartDashboard.putNumber("Eject voltage", FuelConstants.INTAKE_VOLTAGE / 2);
        SmartDashboard.putNumber("Feeder voltage", FuelConstants.FEEDER_VOLTAGE);
        // SmartDashboard.putNumber("Reverse Feeder voltage", FuelConstants.FEEDER_VOLTAGE / 2);

        SmartDashboard.putData("Auto Choices", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driveController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on x press.
        driveController.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        Command elevatorManualUp = elevator.runEnd(
            () -> elevator.setElevatorMotor(11.0),
            () -> elevator.stop()
        );
        Command elevatorManualDown = elevator.runEnd(
            () -> elevator.setElevatorMotor(-11.0),
            () -> elevator.stop()
        );

        driveController.rightBumper().and(driveController.y()).whileTrue(elevatorManualUp);
        driveController.rightBumper().and(driveController.a()).whileTrue(elevatorManualDown);

        drivetrain.registerTelemetry(logger::telemeterize);

        // Binds for shooter and intake, etc.
        manipController.x().whileTrue(new Discharge(shooter));
        manipController.b().whileTrue(new Eject(intake));
        
        manipController.y().onTrue(new ElevatorUp(elevator));
        manipController.a().onTrue(new ElevatorDown(elevator));

        manipController.leftTrigger().toggleOnTrue(new Intake(intake));
        manipController.rightTrigger().whileTrue(new LaunchSequence(shooter));

        manipController.leftBumper().onTrue(new LowerIntake(intakeArm));
        manipController.rightBumper().onTrue(new RaiseIntake(intakeArm));
        
        // manipController.rightBumper().and(driveController.leftBumper()).onTrue();

        /*Should toggle elevator position on press
        manipController.rightTrigger().onTrue(
            new ConditionalCommand(
                new ElevatorUp(elevator),
                new ElevatorDown(elevator), 
                () -> {
                    is_elevator_up = !is_elevator_up;
                    return !is_elevator_up;
                }
            )
        );
        */
    
        //gb 20260306 is_elevator_up starts as false so it is down position initially, 
       
        // add 0.1 debounce to rightTrigger to prevent multiple toggles from a single press, adjust as necessary based on testing
    //    manipController.rightTrigger().debounce(0.1).onTrue(new Elevator(elevator, is_elevator_up));
    /*
        manipController.rightTrigger().debounce(0.1).onTrue(
            new SequentialCommandGroup(  // Toggle the elevator state
                new InstantCommand(() -> {
                    is_elevator_up = !is_elevator_up;
                    SmartDashboard.putBoolean("Is Elevator Up?", is_elevator_up);
                    System.out.println(">>>>>>>>>>ELEVATOR TOGGLED! New value: " + is_elevator_up);
                }),
                new Elevator(elevator, is_elevator_up)
            )
        );
        */
        // Inside RobotContainer.java
    }

    private void consumePoseEstimate(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
        // in meters
        final Translation2d BLUE_HUB = new Translation2d(4.625594, 4.034536);
        final Translation2d RED_HUB = new Translation2d(11.915394, 4.034536);

        Translation2d translation = pose.getTranslation();

        double distanceToBlueHub = translation.getDistance(BLUE_HUB);
        double distanceToRedHub = translation.getDistance(RED_HUB);

        double distanceToUse = Math.min(distanceToBlueHub, distanceToRedHub);
        double idealShootDistance = SmartDashboard.getNumber("Ideal Shoot Distance", 3.0);
        double shootDistanceEpsilon = SmartDashboard.getNumber("Shoot Distance Epsilon", 0.5);

        if (idealShootDistance - shootDistanceEpsilon < distanceToUse && distanceToUse < idealShootDistance + shootDistanceEpsilon) {
            SmartDashboard.putBoolean("At Shoot Distance", true);
        } else {
            SmartDashboard.putBoolean("At Shoot Distance", false);
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // utility function
    private void buildAutoAndAddToChooser(String autoName) {
        autoChooser.addOption(autoName, AutoBuilder.buildAuto(autoName));
    }
}
