package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  DoubleSolenoid solenoidLeft = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, Constants.Pneumatics.extendSolenoidPortLeft, Constants.Pneumatics.retractSolenoidPortLeft);
  DoubleSolenoid solenoidRight = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, Constants.Pneumatics.extendSolenoidPortRight, Constants.Pneumatics.retractSolenoidPortRight);

  public PneumaticsSubsystem() {
    compressor.enableDigital();
  }

  public void extendPiston() {
    if (!compressor.getPressureSwitchValue()) {
      solenoidLeft.set(Value.kForward);
      solenoidRight.set(Value.kReverse);
    }
  }

  public void stopPiston() {
    solenoidLeft.set(Value.kOff);
    solenoidRight.set(Value.kOff);
  }

  public void reversePiston() {
    solenoidLeft.set(Value.kReverse);
    solenoidRight.set(Value.kForward);
  }
}
