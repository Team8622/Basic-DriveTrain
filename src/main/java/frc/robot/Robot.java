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

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

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
  //private final XboxController m_stick = new Joystick(0);

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
    //m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }
}

/*
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  //Joysticks
  Joystick leftJoy = new Joystick(0);
  Joystick rightJoy = new Joystick(1);

  //Motor Controllers (Left Side)
  Spark sparkL1 = new Spark(0);
  Spark sparkL2 = new Spark(1);
  Spark sparkL3 = new Spark(2);

  //Motor Controllers (Right Side)
  Spark sparkR1 = new Spark(3);
  Spark sparkR2 = new Spark(4);
  Spark sparkR3 = new Spark(5);

  SendableChooser<Number> speed = new SendableChooser<>();

  @Override
  public void robotInit() {
    SmartDashboard.putBoolean("Reverse Left Drivetrain?", false);
    SmartDashboard.putBoolean("Reverse Right Drivetrain?", true);
    SmartDashboard.putNumber("Speed Percentage", 100);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    double driveSpeed = SmartDashboard.getNumber("Speed Percentage", 100)/100;
    
  //The robot may drive backwards or spin around due to different drivetrain configurations.
  //Play with these SmartDashboard parameters so it drives correctly.
    if(SmartDashboard.getBoolean("Reverse Left Drivetrain?", true)){
      sparkL1.set(-leftJoy.getY() * driveSpeed);
      sparkL2.set(-leftJoy.getY() * driveSpeed);
      sparkL3.set(-leftJoy.getY() * driveSpeed);
    } else {
      sparkL1.set(leftJoy.getY() * driveSpeed);
      sparkL2.set(leftJoy.getY() * driveSpeed);
      sparkL3.set(leftJoy.getY() * driveSpeed);
    }

    if(SmartDashboard.getBoolean("Reverse Right Drivetrain?", false)){
      sparkR1.set(-rightJoy.getY() * driveSpeed);
      sparkR2.set(-rightJoy.getY() * driveSpeed);
      sparkR3.set(-rightJoy.getY() * driveSpeed);
    } else {
      sparkR1.set(rightJoy.getY() * driveSpeed);
      sparkR2.set(rightJoy.getY() * driveSpeed);
      sparkR3.set(rightJoy.getY() * driveSpeed);
    }
    
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
*/