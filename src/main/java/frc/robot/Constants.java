// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int INTAKE_MOTOR_ID = 52;
    public static final int FEEDER_MOTOR_ID = 53;
    // Gap where follower feeder moter used to be
    public static final int SHOOTER_MOTOR_ID = 55;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 60;
    public static final int SHOOTER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKE_VOLTAGE = 10;
    public static final double FEEDER_VOLTAGE = 9;
    public static final double SHOOTER_VOLTAGE = 10.6;
    public static final double SPIN_UP_SECONDS = 1;
  }

  public static final class IntakeArmConstants {
    public static final int INTAKE_ARM_LEADER_MOTOR_ID = 56;
    public static final int INTAKE_ARM_FOLLOWER_MOTOR_ID = 57;
    
    public static final int INTAKE_ARM_MOTOR_CURRENT_LIMIT = 60;

    public static final double INTAKE_ARM_ENCODER_POSITION_CONVERSION_FACTOR = 1;
    public static final double INTAKE_ARM_ENCODER_VELOCITY_CONVERSION_FACTOR = 1;

    public static final double ZERO_OFFSET = 0;
    public static final double GRAVITY_COMPENSATION = 0.75;

    public static final double INTAKE_ARM_MAX_VELOCITY = 90;
    public static final double INTAKE_ARM_MAX_ACCELERATION = 90;

    public static final double P = 0.012;
    public static final double I = 0;
    public static final double D = 0;

    public static final double INTAKE_ARM_DEFAULT_ANGLE = 55;
    public static final double INTAKE_ARM_MIN_ANGLE = 0;
    public static final double INTAKE_ARM_MAX_ANGLE = 100;

    // May need to merge with min and max angles later
    public static final double INTAKE_ARM_LOWER_ANGLE = 0;
    public static final double INTAKE_ARM_UPPER_ANGLE = 90;
  }

  public static final class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_ID = 51;
    
    public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 60;

    public static final double ELEVATOR_ENCODER_POSITION_CONVERSION_FACTOR = 1;
    public static final double ELEVATOR_ENCODER_VELOCITY_CONVERSION_FACTOR = 1;

    public static final double ZERO_OFFSET = 0;
    public static final double GRAVITY_COMPENSATION = 0.75;

    public static final double ELEVATOR_MAX_VELOCITY = 3.0;
    public static final double ELEVATOR_MAX_ACCELERATION = 3.0;

    public static final double P = 1.0;
    public static final double I = 0;
    public static final double D = 0.01;

    public static final double ELEVATOR_DEFAULT_ROTATION = 1;
    public static final double ELEVATOR_MIN_ROTATION = 0;
    public static final double ELEVATOR_MAX_ROTATION = 10;

    // May need to merge with min and max rotations later
    public static final double ELEVATOR_LOWER_ROTATION = 0;
    public static final double ELEVATOR_UPPER_ROTATION = 8;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .8;
  }
}
