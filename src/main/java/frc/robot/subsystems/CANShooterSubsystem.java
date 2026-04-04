// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANShooterSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax shooterRoller;

  /** Creates a new CANFuelSubsystem. */
  public CANShooterSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    shooterRoller = new SparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);

    feederRoller.configure(
      feederConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.inverted(false);
    shooterConfig.smartCurrentLimit(SHOOTER_MOTOR_CURRENT_LIMIT);
    shooterRoller.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the voltage of the feeder roller
  public void setFeederRoller(double voltage) {
    feederRoller.setVoltage(voltage);
  }

  public void setShooterRoller(double voltage) {
    shooterRoller.setVoltage(voltage);
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    shooterRoller.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
