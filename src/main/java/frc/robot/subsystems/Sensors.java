// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class Sensors extends SubsystemBase {
  /** Creates a new Sensors. */
  private AnalogPotentiometer ultrasonic;

  public Sensors() {
    ultrasonic = new AnalogPotentiometer(0, 100, 0);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ultrasonic", ultrasonic.get() / 0.193);
  }

  public double getUltrasonicDistanceInches(){
    return ultrasonic.get() / 0.193;
  }
}
