package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeArmConstants.*;

public class CANIntakeArmSubsystem extends SubsystemBase {
    private final SparkMax intakeLeaderMotor;
    private final SparkMax intakeFollowerMotor;
    private final RelativeEncoder intakeEncoder;

    // Profiled to ensure smooth movement
    private final ProfiledPIDController intakePIDController;

    private NetworkTableEntry nt_angle, nt_desiredAngle;
    private boolean isDone = false;

    public CANIntakeArmSubsystem() {
        intakeLeaderMotor = new SparkMax(INTAKE_ARM_LEADER_MOTOR_ID, MotorType.kBrushless);
        intakeFollowerMotor = new SparkMax(INTAKE_ARM_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        intakeEncoder = intakeLeaderMotor.getEncoder();

        SparkMaxConfig intakeLeaderMotorConfig = new SparkMaxConfig();
        SparkMaxConfig intakeFollowerMotorConfig = new SparkMaxConfig();

        intakeLeaderMotorConfig
            .smartCurrentLimit(INTAKE_ARM_MOTOR_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        intakeLeaderMotorConfig.encoder
            .positionConversionFactor(INTAKE_ARM_ENCODER_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(INTAKE_ARM_ENCODER_VELOCITY_CONVERSION_FACTOR);

        intakeLeaderMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(P).i(I).d(D);

        intakeLeaderMotor.configure(
            intakeLeaderMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        
        intakeFollowerMotorConfig
            .apply(intakeLeaderMotorConfig)
            .follow(INTAKE_ARM_LEADER_MOTOR_ID);
        
        intakeFollowerMotor.configure(
            intakeFollowerMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        TrapezoidProfile.Constraints intakePIDConstraints;
        // Set both max velocity and max acceleration to 10 deg/sec
        intakePIDConstraints = new TrapezoidProfile.Constraints(
            INTAKE_ARM_MAX_VELOCITY,
            INTAKE_ARM_MAX_ACCELERATION
        );
        
        // Temp PID values, will probably need tuning
        intakePIDController = new ProfiledPIDController(P, I, D, intakePIDConstraints);

        nt_angle = SmartDashboard.getEntry("Intake Angle");
        nt_desiredAngle = SmartDashboard.getEntry("Intake Desired Angle");
        nt_desiredAngle.setDefaultDouble(INTAKE_ARM_DEFAULT_ANGLE);
    }
    
    public double getEncoderAngle() {
        // Converting from rotations to degrees
        return (intakeEncoder.getPosition()*360)%360 - ZERO_OFFSET;
    }

    public void setDesiredAngle(double degrees) {
        nt_desiredAngle.setNumber(degrees);
        isDone = false;
    }

    public boolean isAtDesiredAngle() {
        return isDone;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        final double encoderAngle = getEncoderAngle();

        nt_angle.setDouble(encoderAngle);

        double intakeLeaderMotorSetpoint = MathUtil.clamp(
            nt_desiredAngle.getDouble(INTAKE_ARM_DEFAULT_ANGLE),
            INTAKE_ARM_MIN_ANGLE,
            INTAKE_ARM_MAX_ANGLE
        );

        double intakeLeaderMotorVoltage = /*GRAVITY_COMPENSATION * Math.cos(Math.toRadians(encoderAngle)) 
                                      +*/ intakePIDController.calculate(encoderAngle, intakeLeaderMotorSetpoint);
        
        System.out.print(intakeLeaderMotorVoltage + " ");
        System.out.print(intakeLeaderMotorSetpoint + " ");
        System.out.println(encoderAngle);

        intakeLeaderMotor.setVoltage(intakeLeaderMotorVoltage);
        isDone = Math.abs(encoderAngle - intakeLeaderMotorSetpoint) < 1;
    }
}