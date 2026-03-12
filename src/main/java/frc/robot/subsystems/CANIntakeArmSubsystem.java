package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.IntakeArmConstants.*;

public class CANIntakeArmSubsystem extends SubsystemBase {
    private final SparkMax intakeArmLeaderMotor = new SparkMax(INTAKE_ARM_LEADER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax intakeArmFollowerMotor = new SparkMax(INTAKE_ARM_FOLLOWER_MOTOR_ID, MotorType.kBrushless);

    private final SparkClosedLoopController leaderPidController = intakeArmLeaderMotor.getClosedLoopController();

    // Profiled to ensure smooth movement
    //private final ProfiledPIDController elevatorPIDController;

   // private NetworkTableEntry nt_rotation, nt_desiredRotation;
   // private boolean isDone = false;

    public CANIntakeArmSubsystem() {
    
        SparkMaxConfig intakeArmLeaderMotorConfig = new SparkMaxConfig();
        intakeArmLeaderMotorConfig
        .smartCurrentLimit(INTAKE_ARM_MOTOR_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake)
        .inverted(true);

      // --- PID COEFFICIENTS ---
         // Start with a small P (like 0.1) and increase until it reaches the target
        intakeArmLeaderMotorConfig.closedLoop.p(P); 
        intakeArmLeaderMotorConfig.closedLoop.i(I);
        intakeArmLeaderMotorConfig.closedLoop.d(D);
        intakeArmLeaderMotorConfig.closedLoop.outputRange(-0.5, 0.5); // Limit max speed to 50% for safety

        intakeArmLeaderMotor.configure(intakeArmLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig intakeArmFollowerMotorConfig = new SparkMaxConfig();
        intakeArmFollowerMotorConfig
        .apply(intakeArmLeaderMotorConfig)
        .follow(INTAKE_ARM_LEADER_MOTOR_ID, true);

        intakeArmFollowerMotor.configure(intakeArmFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    };
  

    public void goToPosition(double degrees) {
    // This tells the hardware to handle the movement
    double rotations = (degrees / 360.0) * 50;

    leaderPidController.setReference(rotations, SparkMax.ControlType.kPosition);
    System.out.println("INTAKE ARM MOVING! " + rotations);
  }

  public void stop() {
    intakeArmLeaderMotor.set(0);
  }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    
    }
}