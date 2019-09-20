// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2130.PIDForElevator2019.subsystems;

import org.usfirst.frc2130.PIDForElevator2019.commands.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class elevatorSub extends PIDSubsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private WPI_TalonSRX eLSlave;
    private WPI_TalonSRX eLMaster;
    private DigitalInput maxProx;
    private DigitalInput midProx;
    private DigitalInput lowProx;
    private Solenoid elevatorBrake;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // Initialize your subsystem here
    public elevatorSub() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PID
        super("elevatorSub", 0.001, 0.0, 0.0);
        setAbsoluteTolerance(200);
        getPIDController().setContinuous(false);
        getPIDController().setName("elevatorSub", "PIDSubsystem Controller");
        LiveWindow.add(getPIDController());

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PID

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        /*quadratureEncoder1 = new Encoder(0, 1, false, EncodingType.k4X);
        addChild("Quadrature Encoder 1",quadratureEncoder1);
        quadratureEncoder1.setDistancePerPulse(1.0);
        quadratureEncoder1.setPIDSourceType(PIDSourceType.kRate);*/

        maxProx = new DigitalInput(2);
        midProx = new DigitalInput(1);
        lowProx = new DigitalInput(0);
        
        eLSlave = new WPI_TalonSRX(8);
        
        
        
        eLMaster = new WPI_TalonSRX(9);

        elevatorBrake = new Solenoid(4);
        
        
        

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.

        eLSlave.follow(eLMaster);
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    @Override
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SOURCE
        return encoderValue();

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SOURCE
    }

    @Override
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=OUTPUT
        eLMaster.pidWrite(-output);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=OUTPUT
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    public void zeroTheEncoder() {
        eLMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public void homEncoder() {
        if (!lowProx.get()) {
            zeroTheEncoder();
        }
    }

    public boolean maxProxValue() {
        return !maxProx.get();
    }

    public double encoderValue() {
        return -eLMaster.getSelectedSensorPosition(0);
    }

    public void brakeState(boolean state) {
        elevatorBrake.set(state);
    }

}