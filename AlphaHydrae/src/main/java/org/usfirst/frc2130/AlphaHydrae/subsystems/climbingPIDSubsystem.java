package org.usfirst.frc2130.AlphaHydrae.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.usfirst.frc2130.AlphaHydrae.Robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class climbingPIDSubsystem extends PIDSubsystem {

    private WPI_TalonSRX rearLiftMotor;
    private WPI_TalonSRX rearFootMotor;
    private DoubleSolenoid climbingFeetSolenoid;
    private DigitalInput footProx;

    private double collectiveOutput;
    private boolean atClimbableLevel;

    // Initialize your subsystem here
    public climbingPIDSubsystem() {
        super("climbingPIDSubsystem", 0.05, 0.0, 0.0);

        // Initialize collective output to disable climbing
        collectiveOutput = 0.0;

        setAbsoluteTolerance(0.2);
        getPIDController().setContinuous(false);
        getPIDController().setName("climbingPIDSubsystem", "PIDSubsystem Controller");
        LiveWindow.add(getPIDController());
        
        rearLiftMotor = new WPI_TalonSRX(7);
        rearFootMotor = new WPI_TalonSRX(6);

        footProx = new DigitalInput(3);

        climbingFeetSolenoid = new DoubleSolenoid(0, 5, 6);

        // Make sure the system is off to start with
        disable();
    }

    @Override
    public void initDefaultCommand() {
        // Unused. No default command should be set for this subsystem.
    }

    @Override
    protected double returnPIDInput() {
        // This may need to be inverted to get the correct direction.
        //
        // Assume here that positive pitch means the the front of the robot
        // is tilted up, that means that the front motor needs to slow down.
        return Robot.navXSub.navXPitch();

    }

    public double liftingModifier() {
        return Robot.navXSub.navXPitch() / 100;
    }

    @Override
    protected void usePIDOutput(double output) {
        // The PID output represents the controller's attempt to compensate for
        // perceived pitch error. Following our convention for positive pitch
        // meaning the robot is tilted up, for a simple P contoller:
        //
        //      output = Kp * error = Kp * (target - actual)
        //      output = Kp * (0 - pitch)
        //
        // So we see that if there is positive pitch, the controller output will
        // be negative. That means we should ADD the output of the controller to the
        // front motor and SUBTRACT the output from the back motor.

        // Add controller output on front to decrease speed if pitch is positive
        //Robot.elevatorPIDSubsystem.applyPower(collectiveOutput - output);

        // Subtract controller output on back to increase speed if pitch is positive
        //rearLiftMotor.set((-collectiveOutput) + output * 4);
        
    }

    public void startClimb(double collectiveCommand) {
        // Starts climbing by setting the collective command and enabling the controller.
        //
        // By default, we target 0 tilt. If we want to get fancy we may want to maintain the
        // orientation the robot is when it starts to climb by remembering the pitch when the
        // command is sent. It may be more desirable for it to self-level in the end though.

        // Target 0 pitch
        setSetpoint(0.0);

        // Set the internal collective output
        collectiveOutput = collectiveCommand;

        

        // Start the PID controller
        enable();
    }

    public double encoderPosition() {
        return Robot.elevatorPIDSubsystem.elevatorEncoderValue();
    }

    public void resetFeet() {
        climbingFeetSolenoid.set(Value.kForward);
    }

    public boolean atSetpoint() {
        return atClimbableLevel;
    }

    public boolean atFootProx() {
        return footProx.get();
    }

    public void liftElevator(int setpoint, int error) {

        Robot.elevatorPIDSubsystem.setMaxMinOutput(0.7, -0.1);

        if (encoderPosition() >= setpoint + error) {
            Robot.elevatorPIDSubsystem.applyPower(0);
            atClimbableLevel = false;
        } else if (encoderPosition() > setpoint - error && encoderPosition() < setpoint + error) {
            Robot.elevatorPIDSubsystem.applyPower(0);
            atClimbableLevel = true;
        } else {
            Robot.elevatorPIDSubsystem.applyPower(1);
            Robot.elevatorPIDSubsystem.setBrakeState(true);
            atClimbableLevel = false;
        }

        Robot.clawSubsystem.lowerClaw();
        
    }

    public void stopAllMotors() {
        Robot.elevatorPIDSubsystem.stopAllMotors();
        rearFootMotor.set(0);
        rearLiftMotor.set(0);
        Robot.driveTrainSubsystem.stopAllMotors();
    }

    public void disengagaeBrakes() {
        Robot.elevatorPIDSubsystem.setBrakeState(true);
    }

    public void endElevator() {
        Robot.elevatorPIDSubsystem.setBrakeState(false);
        climbingFeetSolenoid.set(Value.kReverse);
    }

    public int getEncoderPosition() {
        return -rearFootMotor.getSelectedSensorPosition(0);
    }

    public void zeroEncoderPosition() {
        rearFootMotor.setSelectedSensorPosition(0, 0, 0);
    }

    public void liftRobot() {

        Robot.elevatorPIDSubsystem.setMaxMinOutput(0.3, -0.35);

        rearLiftMotor.configPeakOutputForward(1, 0);
        rearLiftMotor.configPeakOutputReverse(-1, 0);

        rearFootMotor.configPeakOutputForward(1, 0);
        rearFootMotor.configPeakOutputReverse(-1, 0);

        if (this.getEncoderPosition() < 27000 && this.getEncoderPosition() > 0) {
            Robot.elevatorPIDSubsystem.setBrakeState(true);
            Robot.elevatorPIDSubsystem.applyPower(-1);
            rearLiftMotor.set(-1);
            Robot.clawSubsystem.raiseClaw();
            rearFootMotor.set(-0.1);
        } else {
            Robot.elevatorPIDSubsystem.setBrakeState(false);
            Robot.elevatorPIDSubsystem.applyPower(0);
            rearLiftMotor.set(0);
            Robot.clawSubsystem.raiseClaw();
            rearFootMotor.set(-0.1);
        }

        Robot.elevatorPIDSubsystem.setBrakeState(true);
        Robot.elevatorPIDSubsystem.applyPower(-1);
        rearLiftMotor.set(-1);
        Robot.clawSubsystem.raiseClaw();
        rearFootMotor.set(-0.1);
    }

    public void retractFoot() {
        rearLiftMotor.set(1);
    }

    public void moveRobot() {
        rearFootMotor.set(1);
        Robot.driveTrainSubsystem.applyPower(0.2);
    }

    public boolean atBottomProx() {
        return Robot.elevatorPIDSubsystem.getProx("Low");
    }

    public boolean atMiddleProx() {
        return Robot.elevatorPIDSubsystem.getProx("Mid");
    }

    public void engageBrakes() {
        Robot.elevatorPIDSubsystem.setBrakeState(false);
    }

    public void startup() {
        //Robot.elevatorPIDSubsystem.setBrakeState(true);
        zeroEncoderPosition();
        climbingFeetSolenoid.set(Value.kForward);
        atClimbableLevel = false;
    }

    /*
    public void stopClimb() {
        // Turn everything off
        Robot.elevatorPIDSubsystem.setBrakeState(false);
        disable();
        rearLiftMotor.set(0.0);
        Robot.elevatorPIDSubsystem.stopAllMotors();;
        collectiveOutput = 0.0;
    }

    /*public double footEncoderValue() {
        return -rearLiftMotor.getSelectedSensorPosition(0);
    }*

    public void zeroEncoder() {
        rearLiftMotor.setSelectedSensorPosition(0,0,0);
    }

    public void prepareToClimb() {
        rearLiftMotor.configPeakOutputForward(1, 0);
        rearLiftMotor.configPeakOutputReverse(-1, 0);
    }

    public boolean getProx(boolean useTopProx) {
        if (useTopProx) {
            return !topProx.get();
        }  else {
            return !bottomProx.get();
        }
    }

    public void homeEncoder() {
    	if (getProx(false) == true) {
    	    zeroEncoder();
    	}
    }

    public boolean isAtMovableLevel() {

        //if (getProx(true)) {
        //    return true;
        //}

        if (Robot.elevatorPIDSubsystem.getProx("Low")) {
            return true;
        } else {
            return false;
        }

        /*if (footEncoderValue() <= -26169) {
            return true;
        } else {
            return false;
        }*
    }

    public void zeroNavx() {
        
    }

    public boolean retractedLevel() {

        if (!getProx(false)) {
            return true;
        } else { 
            return false;
        }

        /*if (footEncoderValue() <= 10) {
            return true;
        } else {
            return false;
        }*
    }

    public void returnFoot() {
        rearLiftMotor.set(0.5);
    }

    public void killLifterFoot() {
        rearLiftMotor.set(0);
    }*/

}
