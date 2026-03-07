package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor = new SparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
    private final SparkClosedLoopController pidController = elevatorMotor.getClosedLoopController();

    // Profiled to ensure smooth movement
    //private final ProfiledPIDController elevatorPIDController;

   // private NetworkTableEntry nt_rotation, nt_desiredRotation;
   // private boolean isDone = false;

    public ElevatorSubsystem() {
    
        SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();

        elevatorMotorConfig
        .smartCurrentLimit(ELEVATOR_MOTOR_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake)
        .inverted(false);

      // --- PID COEFFICIENTS ---
         // Start with a small P (like 0.1) and increase until it reaches the target
        elevatorMotorConfig.closedLoop.p(0.1); 
        elevatorMotorConfig.closedLoop.i(0);
        elevatorMotorConfig.closedLoop.d(0);
        elevatorMotorConfig.closedLoop.outputRange(-0.5, 0.5); // Limit max speed to 50% for safety

        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    };

    public void goToPosition(double rotations) {
    // This tells the hardware to handle the movement
    pidController.setReference(rotations, SparkMax.ControlType.kPosition);
     System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>ELEVATOR MOVING! "+rotations);
     
  }

  public void stop() {
    elevatorMotor.set(0);
  }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    
    }
}