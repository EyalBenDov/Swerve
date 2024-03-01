// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  private final CANSparkMax grabFeedMotor;
  private final CANSparkMax moveFeedMotor;

  private final CANSparkMax launchMotor1;
  private final CANSparkMax launchMotor2;
  /** Creates a new ExampleSubsystem. */

  public Launcher() {
    moveFeedMotor = new CANSparkMax(18, MotorType.kBrushless);
    grabFeedMotor = new CANSparkMax(13, MotorType.kBrushless);
    launchMotor1 = new CANSparkMax(11, MotorType.kBrushless);
    launchMotor2 = new CANSparkMax(12, MotorType.kBrushless);

    grabFeedMotor.restoreFactoryDefaults();
    moveFeedMotor.restoreFactoryDefaults();
    launchMotor1.restoreFactoryDefaults();
    launchMotor2.restoreFactoryDefaults();
  }

  public void setGrabMotors() {
    grabFeedMotor.set(0.5);
    moveFeedMotor.set(0.5);
    System.out.println("Motor should have moved!");
    System.out.println("moveFeedMotor.get(): " + moveFeedMotor.get());
  }

  public void setLaunchMotors() {
    launchMotor1.set(0.6);
    launchMotor2.set(0.6);
  }

  public void stopGrabMotors() {
    grabFeedMotor.set(0);
    moveFeedMotor.set(0);
  }

  public void stopLaunchMotors() {
    launchMotor1.set(0);
    launchMotor2.set(0);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
