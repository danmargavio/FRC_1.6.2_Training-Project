// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  /**
   * This program demonstrates a basic time-based robot employing a velocity controlled single NEO motor program.
   */

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

// We add this line to enable us to call and use a Joystick
import edu.wpi.first.wpilibj.Joystick;

// We add this line to enable us to call and use a REV Robotics Spark Max motor controller using the REV Robotics library
import com.revrobotics.CANSparkMax;

// We add this line to enable us to call and new data type corresponding to the motor we are using
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// We add this line to allow creation of a motor encoder object for tracking the motor position
import com.revrobotics.RelativeEncoder;

// We add this line to allow creation of a Spark PID Controller object for closed loop control of the motor
import com.revrobotics.SparkMaxPIDController;

public class Robot extends TimedRobot {

  // We add this line to create a new instance of a joystick object connected to USB port 0
  Joystick my_joystick = new Joystick(0);

  // We add this line to create a new instance of a motor controller object connected to CAN bus port 20 and using a brushless type motor (aka Neo)
  CANSparkMax my_motor_controller = new CANSparkMax(20, MotorType.kBrushless);

  // We assign a new PID controller object to the existing motor controller object's PID controller we created
  SparkMaxPIDController m_pidController = my_motor_controller.getPIDController();

  // We create an Encoder object just for display purposes
  RelativeEncoder m_encoder = my_motor_controller.getEncoder();

  // We add this line to create a new instance of a "double" variable and set its value to 0
  double motor_control_input = 0.0;

  // PID coefficients
  double kP = 6e-5; 
  double kI = 0;
  double kD = 0; 
  double kIz = 0; 
  double kFF = 0; 
  double kMaxOutput = 1; 
  double kMinOutput = -1;
  double maxRPM = 5700;
  
  @Override
  public void robotInit() {
    // This is a good idea; it "resets" the Spark Max to factory default settings
    my_motor_controller.restoreFactoryDefaults();

  // set PID coefficients
  m_pidController.setP(kP);
  m_pidController.setI(kI);
  m_pidController.setD(kD);
  m_pidController.setIZone(kIz);
  m_pidController.setFF(kFF);
  m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void robotPeriodic() {
    // Get the current value of my joysticks left controller Y axis and set my motor_control_input variable to it
    motor_control_input = maxRPM * my_joystick.getRawAxis(1); // multiple by maxRPM so that the velocity never exceeds that
  }

  @Override
  public void teleopPeriodic() {
    // set the motor PID controller velocity control input to the motor control input
    m_pidController.setReference(motor_control_input, CANSparkMax.ControlType.kVelocity);
  }
}