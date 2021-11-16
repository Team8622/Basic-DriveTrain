/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Sourced from WPILib's Arcade Drive example, Rev's CAN Spark example, and some guessing
 */
public class Robot extends TimedRobot {

  //Motor Controller CAN Ids
  public static final int leftID1 = 1;
  public static final int leftID2 = 2;
  public static final int rightID1 = 3;
  public static final int rightID2 = 4;
  
  //Left Side Motor Controllers
  private final CANSparkMax m_leftlead = new CANSparkMax(leftID1, MotorType.kBrushless);  
  private final CANSparkMax m_leftfollow = new CANSparkMax(leftID2, MotorType.kBrushless);
  
  //Right Side Motor Controllers
  private final CANSparkMax m_rightlead = new CANSparkMax(rightID1, MotorType.kBrushless);
  private final CANSparkMax m_rightfollow = new CANSparkMax(rightID2, MotorType.kBrushless);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftlead, m_rightlead);
  private final Joystick m_stick = new Joystick(0);

  @Override
  public void robotInit() {
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_leftfollow.follow(m_leftlead);
    m_rightfollow.follow(m_rightlead);
    m_robotDrive.arcadeDrive(-m_stick.getY() * 0.75, m_stick.getX());
  }
}