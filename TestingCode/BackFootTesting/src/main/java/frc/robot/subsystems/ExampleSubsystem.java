/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.ExampleCommand;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class ExampleSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_TalonSRX footMotor;
  public Solenoid brake;

  public ExampleSubsystem() {
    footMotor = new WPI_TalonSRX(7);
    brake = new Solenoid(4);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new ExampleCommand());
  }

  public int getEncoderPosition() {
    return footMotor.getSelectedSensorPosition(0);
  }

  public void zeroEncoderPosition() {
    footMotor.setSelectedSensorPosition(0, 0, 0);
  }

  public void moveFoot() {
    footMotor.set(Robot.oi.driver.getRawAxis(1)*-1);
    brake.set(true);
  }

  public void stopfoot() {
    footMotor.set(0);
    brake.set(false);
  }

}
