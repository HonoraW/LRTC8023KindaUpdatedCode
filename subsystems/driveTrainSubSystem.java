// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

public class driveTrainSubSystem extends SubsystemBase {
  CANSparkMax leftFrontMotor = new CANSparkMax(Constants.driveTrainConstants.leftFrontCANID, CANSparkMaxLowLevel.MotorType.kBrushless);  // SPARK MAX for left front NEO motor.
  CANSparkMax leftBackMotor = new CANSparkMax(Constants.driveTrainConstants.leftBackCANID, CANSparkMaxLowLevel.MotorType.kBrushless); // SPARK MAX for left Back NEO motor.
  CANSparkMax rightFrontMotor = new CANSparkMax(Constants.driveTrainConstants.rightFrontCANID, CANSparkMaxLowLevel.MotorType.kBrushless); // SPARK MAX for right Front NEO motor.
  CANSparkMax rightBackMotor = new CANSparkMax(Constants.driveTrainConstants.rightBackCANID, CANSparkMaxLowLevel.MotorType.kBrushless); // SPARK MAX for right Back NEO Motor


  RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
  RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);
//Gyroscope object
  public final static Gyro adis16470 = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new ExampleSubsystem. */
  public driveTrainSubSystem() {
    // restore factory defaults for all motors
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults(); 
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    

    // set the values of the encoders to zero when robot starts
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    
    rightEncoder.setPositionConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor);
    rightEncoder.setVelocityConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor/60);
    leftEncoder.setVelocityConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor/60);

    // make motors follow each other
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    rightControllerGroup.setInverted(false);
    leftControllerGroup.setInverted(true);

    adis16470.reset();
    adis16470.calibrate();
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(adis16470.getRotation2d());
    m_odometry.setPosition(new Pose2d(), adis16470.getRotation2d())
  }

  public void setBreakMode(){
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    leftFrontMotor.setIdleMode(IdleMode.kBreak);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBreak);
  }

  public void setCoastMode(){
    leftBackMotor.setIdleMode(IdleMode.kCoast);
    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightBackMotor.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

    public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public double getRightEncoderPosition(){
    return rightEncoder.getPosition();
  }

  public double getLeftEncoderPosition(){
    return leftEncoder.getPosition();
  }

  public double getRightEncoderVelocity(){
    return rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity(){
    return leftEncoder.getVelocity();
  }

  public static double getHeading(){
    return adis16470.getRotation2d().getDegrees();
  }

  public double getTurnRate(){
    return -adis16470.getRate();
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(pose, adis16470.getRotation2d())
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftControllerGroup.setVoltage(leftVolts);
    rightControllerGroup.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public double getAverageEncoderDistance(){
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  public RelativeEncoder getLeftEncoder(){
    return leftEncoder;
  }

  output

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // getting position in encoder ticks
    m_odometry.update(adis16470.getRotation2d(), leftEncoder.getPosition(), righEncoder.getPosition());
    //put info on smart dashboard
    SmartDashboard.putNumber("Left Encoder value meters", getLeftEncoderPosition())
    SmartDashboard.putNumber("Right Encoder value meters", getRightEncoderPosition())
    SmartDashboard.putNumber("Gyro heading", getHeading());
  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
