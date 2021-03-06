// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.util.PIDSparkMotor;
import frc.robot.Constants.ChassisConstants;


// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Chassis extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private CANSparkMax frontLeftMotor;
    private CANSparkMax rearLeftMotor;
    private CANSparkMax frontRightMotor;
    private CANSparkMax rearRightMotor;

    private PIDSparkMotor PIDFrontLeftMotor;
    private PIDSparkMotor PIDFrontRightMotor;

    private DifferentialDrive differentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
    *
    */
    public Chassis() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        frontLeftMotor = new CANSparkMax(2, MotorType.kBrushless);
        frontLeftMotor.restoreFactoryDefaults();
        frontLeftMotor.setInverted(false);
        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        PIDFrontLeftMotor = new PIDSparkMotor(frontLeftMotor, ChassisConstants.kP, ChassisConstants.kI, ChassisConstants.kD);

        frontRightMotor = new CANSparkMax(4, MotorType.kBrushless);
        frontRightMotor.restoreFactoryDefaults();
        frontRightMotor.setInverted(false);
        frontRightMotor.setIdleMode(IdleMode.kBrake);
        PIDFrontRightMotor = new PIDSparkMotor(frontRightMotor, ChassisConstants.kP, ChassisConstants.kI, ChassisConstants.kD);

        rearLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
        rearLeftMotor.restoreFactoryDefaults();
        rearLeftMotor.setInverted(false);
        rearLeftMotor.setIdleMode(IdleMode.kBrake);
        rearLeftMotor.follow(frontLeftMotor);

        rearRightMotor = new CANSparkMax(5, MotorType.kBrushless);
        rearRightMotor.restoreFactoryDefaults();
        rearRightMotor.setInverted(false);
        rearRightMotor.setIdleMode(IdleMode.kBrake);
        rearRightMotor.follow(frontRightMotor);

        differentialDrive = new DifferentialDrive(PIDFrontLeftMotor, PIDFrontRightMotor);
        addChild("differentialDrive", differentialDrive);
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(1.0);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void drive(double speed, double rotation){
        differentialDrive.arcadeDrive(speed, rotation);
    }

    public void setBrake(boolean brake){
        if (brake){
            rearRightMotor.setIdleMode(IdleMode.kBrake);
            rearLeftMotor.setIdleMode(IdleMode.kBrake);
            frontRightMotor.setIdleMode(IdleMode.kBrake);
            frontLeftMotor.setIdleMode(IdleMode.kBrake);
        } else {
            rearRightMotor.setIdleMode(IdleMode.kCoast);
            rearLeftMotor.setIdleMode(IdleMode.kCoast);
            frontRightMotor.setIdleMode(IdleMode.kCoast);
            frontLeftMotor.setIdleMode(IdleMode.kCoast);
        }
    }
}