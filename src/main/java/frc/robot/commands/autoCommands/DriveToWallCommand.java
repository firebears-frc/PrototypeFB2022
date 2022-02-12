// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToWallCommand extends PIDCommand {
  /** Creates a new driveToWallCommand. */
  Sensors m_sensors;
  Chassis m_chassis;

  public DriveToWallCommand(double distance, Sensors sensors, Chassis chassis) {
    super(
        // The controller that the command will use
        new PIDController(0.118, 0, 0),
        // This should return the measurement
        () -> sensors.getUltrasonicDistanceInches(),
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          chassis.drive(0, -0.5 * MathUtil.clamp(output, -1.0, 1.0));
          System.out.println("Distance: " + sensors.getUltrasonicDistanceInches() + 
                             "Output" + -0.25 * output);
        });

      addRequirements(sensors, chassis);
      getController().setTolerance(0.5, 0.5);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}