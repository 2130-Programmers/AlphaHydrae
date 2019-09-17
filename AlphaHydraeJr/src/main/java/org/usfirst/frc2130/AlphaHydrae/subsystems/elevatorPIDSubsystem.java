package org.usfirst.frc2130.AlphaHydrae.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc2130.AlphaHydrae.Robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class elevatorPIDSubsystem extends PIDSubsystem {

    private DigitalInput lowProx;
    private DigitalInput midProx;
    private DigitalInput maxProx;
    private WPI_TalonSRX elevatorMotorMaster;
    private WPI_TalonSRX elevatorMotorSlave;
    private Solenoid elevatorBrakeSolenoid;
    private String desiredProx;
    public boolean atClimbingSetpoint;
    public boolean endExtraction;

    //public int i;

    // Initialize your subsystem here
    public elevatorPIDSubsystem() {
        super("elevatorPIDSubsystem", 0.0004, 0.0, 0.0);
        setAbsoluteTolerance(100);
        getPIDController().setContinuous(false);
        getPIDController().setName("elevatorPIDSubsystem", "PIDSubsystem Controller");
        LiveWindow.add(getPIDController());
    
        // Make sure the system is off to start with
        //disable();

        atClimbingSetpoint = false;

        //i = 0;

        lowProx = new DigitalInput(0);
        addChild("lowProx",lowProx);
        
        
        midProx = new DigitalInput(1);
        addChild("midProx",midProx);
        
        
        maxProx = new DigitalInput(2);
        addChild("maxProx",maxProx);
        
        
        elevatorMotorMaster = new WPI_TalonSRX(8);
        
        elevatorMotorSlave = new WPI_TalonSRX(9);

        elevatorBrakeSolenoid = new Solenoid(4);

        elevatorMotorMaster.setInverted(true);
        elevatorMotorSlave.setInverted(true);

        followMaster();

        desiredProx = "Low";
    }

    @Override
    public void initDefaultCommand() {}

    @Override
    protected double returnPIDInput() {
        return elevatorEncoderValue();
    }

    @Override
    protected void usePIDOutput(double output) {
        elevatorMotorMaster.set(output);
    }

    /** Dynamic Methods */
    /* These are the values we send outside the subsystem */

    public double elevatorEncoderValue() {

        // Calls the elevator motors to return us the encoder value at it's current state

        return elevatorMotorSlave.getSelectedSensorPosition(0);
    }

    public double motorOutput() {

        // Calls the current percentage of voltage being provided to the elevator motors

    	return elevatorMotorMaster.getMotorOutputPercent();
    }

    public double motorSlaveOutput() {

        // Calls the current percentage of voltage being provided to the elevator motors

    	return elevatorMotorSlave.getMotorOutputPercent();
    }

    public double getJoystick(int axis) {

        // Calls a joystick and inverts it

        return -Robot.oi.operatorJoystick.getRawAxis(axis);
    }

    public boolean getProx(String prox) {

        // Returns the current state of a given prox

        if (prox == "Max") {
            return !maxProx.get();
        } else if (prox == "Mid") {
            return !midProx.get();
        } else {
            return !lowProx.get();
        }
    }

    /** Startup Tasks */
    /* These are the fuctions we use when we need to do a specific thing at the start of a match or command */

    public void resetpeakoutput() {

        // This resets the outputs to be full power in both directions
        // This can create oscialtion (or unexpected speed) if called before a PID method

    	setMaxMinOutput(1, -1);
    }

    
    public void zeroTheTalon() {

        // Resets the elevator encoder to be at zero

    	elevatorMotorSlave.setSelectedSensorPosition(0, 0, 0);
    }


    /** Basic Fuctionality */
    /* These are the functions we use to do basic tasks like killing the motors for example */

    public void setBrakeState (boolean on) {
        elevatorBrakeSolenoid.set(on);
    }

    public void stopAllMotors() {
        elevatorMotorMaster.set(0);
        elevatorMotorSlave.set(0);
    }
    
    public void homeEncoder() {
    	if (getProx("Low")) {
    	    zeroTheTalon();
    	}
    }





    

    public String returnDesiredProx() {
        return desiredProx;
    }

    public void startupRoutine() {
        setBrakeState(false);
        desiredProx = "Low";
        setMaxMinOutput(0.7, -0.1);
        followMaster();
        endExtraction = false;
    }

    public void moveElevator() {{
        if (getProx(desiredProx)) {
            disableElevator();
        } else {
            if (elevatorEncoderValue() < 8000) {
                setMaxMinOutput(0.7, -0.1);
            }
            if (elevatorEncoderValue() >= 8000 && elevatorEncoderValue() < 13000) {
                setMaxMinOutput(0.7, -0.2);
            }
            if (elevatorEncoderValue() >= 13000) {
                setMaxMinOutput(0.5, -0.2);
            }
            setBrakeState(true);
            elevatorMotorMaster.set(getJoystick(1));
            } 
        }
    }



    public void atTopProx() {
        stopAllMotors();
        setBrakeState(false);
    }

    public void applyPower(double speed) {
        elevatorMotorMaster.set(speed);
    }

    public void prepareToClimb() {
        setMaxMinOutput(0.7, -0.25);
    }


    public void setMaxMinOutput(double max, double min) {
        elevatorMotorMaster.configPeakOutputForward(max, 0);
        elevatorMotorMaster.configPeakOutputReverse(min, 0);
        elevatorMotorSlave.configPeakOutputForward(max, 0);
        elevatorMotorSlave.configPeakOutputReverse(min, 0);
    }

    // Disables the elevator, and turns on the brake
    public void disableElevator() {
        stopAllMotors();
        setBrakeState(false);
    }

    public void setDesiredOutput(String setDesiredProx) {
        desiredProx = setDesiredProx;
    }

    public boolean isAtLimit() {
        if (motorOutput() > 0 && getProx("Max")) {
            return true;
        } else if (motorOutput() < 0 && getProx("Low")) {
            return true;
        } else {
            return false;
        }

    }
    
    public void setSetpointWithBrake(int setpoint) {    
    	/* This code functions as a sort of dampener. When the elevator is below a certain point it slows down and
    	 * gently parks at the bottom prox */
    	if (elevatorEncoderValue() < 4000) {
            setMaxMinOutput(0.6, -0.1);
    	}
    	if (elevatorEncoderValue() >= 4000) {
            setMaxMinOutput(0.6, -0.2);
        }
        if (elevatorEncoderValue() >= 13000) {
            setMaxMinOutput(0.4, -0.2);
        }
    	
    	/* Currently this gives our loop a tolerance of 10 native encoder units. Once we are within this range,
    	 * the brake will engage and the loop will end, holding us position until a new setpoint is called*/
    	if(elevatorEncoderValue() > setpoint - 500 && elevatorEncoderValue() < setpoint + 500 && elevatorEncoderValue() > 1000) {
    		disable();
    		setBrakeState(false);
    		stopAllMotors();
    	}
    	else {
            setSetpoint(setpoint);
    		enable();
    		setBrakeState(true);
    	}
    	
    	//if(getProx("Low") && setpoint == 120) {
    	//	disable();
    	//	stopAllMotors();
        //}
    }

    public void followMaster() {
        elevatorMotorSlave.follow(elevatorMotorMaster);
    }

    public boolean returnAtClimbingSetpoint() {
        return atClimbingSetpoint;
    }

    public void climbingSetpointWithBrake(int setpoint) {    
    	/* This code functions as a sort of dampener. When the elevator is below a certain point it slows down and
    	 * gently parks at the bottom prox */
    	if (elevatorEncoderValue() < 4000) {
            setMaxMinOutput(0.6, -0.1);
    	}
    	if (elevatorEncoderValue() >= 4000) {
            setMaxMinOutput(0.4, -0.2);
        }
    	
    	if(motorOutput() < 0 && getProx("Max") == true) {
    		disable();
    		elevatorMotorMaster.set(0);
    	}
    	if(motorOutput() > 0 && getProx("Low") == true) {
    		disable();
    		elevatorMotorMaster.set(0);
        }
        if(motorOutput() != 0 && elevatorEncoderValue() > setpoint + 750) {
            elevatorMotorMaster.set(0);
            setBrakeState(false);
        }
    	
    	/* Currently this gives our loop a tolerance of 10 native encoder units. Once we are within this range,
    	 * the brake will engage and the loop will end, holding us position until a new setpoint is called*/
    	if(elevatorEncoderValue() > setpoint - 500 && elevatorEncoderValue() < setpoint + 500 && elevatorEncoderValue() > 1000) {
            setBrakeState(false);
            atClimbingSetpoint = true;
    	}
    	else {
            setSetpoint(setpoint);
    		enable();
            //setBrakeState(true);
            atClimbingSetpoint = false;
    	}
    	
    	//if(getProx("Low") && setpoint == 120) {
    	//	disable();
    	//	stopAllMotors();
        //}
    }

    public boolean endExtractionValue() {
        return endExtraction;
    }
    /*
   public boolean setpointCheck(int setpoint) {

        isAtLimit();

        if (elevatorEncoderValue() < (setpoint + 200) && elevatorEncoderValue() > (setpoint - 200)) {
            return true;
        } else {
            return false;
        }
    }*/
    
}
