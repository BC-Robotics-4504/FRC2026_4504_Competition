// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_MAX_ROTATION;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_MIN_ROTATION;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Discharge;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
// import frc.robot.commands.Launch;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.LowerIntake;
import frc.robot.commands.RaiseIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANElevatorSubsystem;
import frc.robot.subsystems.CANIntakeArmSubsystem;
// import frc.robot.subsystems.CANElevatorSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANSwerveSubsystem;
// import frc.robot.Constants
import frc.robot.Constants.*;

public class RobotContainer {
    // Changed 1.0 to 0.3
    private double MaxSpeed = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
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

    public final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();

    public final CANElevatorSubsystem elevator = new CANElevatorSubsystem();
    // public final CANElevatorSubsystem elevator = new CANElevatorSubsystem();

    public final CANIntakeArmSubsystem intakeArm = new CANIntakeArmSubsystem();

    private final SendableChooser<Command> autoChooser  = new SendableChooser<>();

    private boolean is_elevator_up = false;

    public RobotContainer() {
        configureBindings();

        autoChooser.addOption("Fancy Test Auto", AutoBuilder.buildAuto("Fancy Test Auto"));
        autoChooser.setDefaultOption("4m Foreward Auto", AutoBuilder.buildAuto("4m Foreward Auto"));

        // TODO: make commands for Intake and Shooter (FOR PATHPLANNER)
        NamedCommands.registerCommand("Shoot", new LaunchSequence(fuelSubsystem));
        NamedCommands.registerCommand("Intake Down", new LowerIntake(intakeArm));

        SmartDashboard.putNumber("Shooter voltage", FuelConstants.SHOOTER_VOLTAGE);
        SmartDashboard.putNumber("Intake voltage", FuelConstants.INTAKE_VOLTAGE);
        SmartDashboard.putNumber("Eject voltage", FuelConstants.INTAKE_VOLTAGE / 2);
        SmartDashboard.putNumber("Feeder voltage", FuelConstants.FEEDER_VOLTAGE);
        SmartDashboard.putNumber("Reverse Feeder voltage", FuelConstants.FEEDER_VOLTAGE / 2);

        SmartDashboard.putData("Auto Choices", autoChooser);
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

        // TODO: shoot distance code (MAYBE center to hub)

        drivetrain.registerTelemetry(logger::telemeterize);

        // Binds for shooter and intake, etc.
        manipController.rightTrigger().whileTrue(new LaunchSequence(fuelSubsystem));
        manipController.b().whileTrue(new Discharge(fuelSubsystem));
        manipController.x().whileTrue(new Eject(fuelSubsystem));

        manipController.leftTrigger().toggleOnTrue(new Intake(fuelSubsystem));

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

        // COMMAND: Go to Top (8 rotations)
        Command elevatorTop = Commands.runOnce(() -> elevator.goToPosition(ELEVATOR_MAX_ROTATION));
        //Command elevatorTop = Commands.runOnce(() -> elevator.goToPosition(ELEVATOR_ROTATIONS));

        // COMMAND: Go to Bottom (0 rotations)
        Command elevatorBottom = Commands.runOnce(() -> elevator.goToPosition(ELEVATOR_MIN_ROTATION));

        manipController.a().onTrue(elevatorTop);
        manipController.y().onTrue(elevatorBottom);

        manipController.leftBumper().onTrue(new LowerIntake(intakeArm));
        manipController.rightBumper().onTrue(new RaiseIntake(intakeArm));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
