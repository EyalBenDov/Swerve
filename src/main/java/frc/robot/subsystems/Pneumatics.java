// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final Compressor m_compressor;
  private final DoubleSolenoid liftSolenoid;
  private final DoubleSolenoid blockSolenoid;
  public Pneumatics() {
    m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    blockSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
    liftSolenoid= new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  }
  public void startCompressor() {
    m_compressor.enableDigital();
  }

  public void toggleLiftSolenoid() {
    liftSolenoid.toggle();
  }
  public void disableLiftSolenoid() {
    liftSolenoid.set(DoubleSolenoid.Value.kOff);
  }
  public void forwardLiftSolenoid() {
    liftSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  public void reverseLiftSolenoid() {
    liftSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  
  public void toggleBlockSolenoid() {
    blockSolenoid.toggle();
  }
  public void disableBlockSolenoid() {
    blockSolenoid.set(DoubleSolenoid.Value.kOff);
  }
  public void forwardBlockSolenoid() {
    System.out.println("FORWARD BLOCK SOLENOID RAN!");
    blockSolenoid.set(DoubleSolenoid.Value.kForward);
    System.out.println("pressureValue: " + m_compressor.getPressureSwitchValue());
  }
  public void reverseBlockSolenoid() {
    blockSolenoid.set(DoubleSolenoid.Value.kReverse);
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
