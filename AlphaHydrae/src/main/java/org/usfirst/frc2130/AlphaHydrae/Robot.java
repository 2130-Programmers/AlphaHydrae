// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2130.AlphaHydrae;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc2130.AlphaHydrae.commands.*;
import org.usfirst.frc2130.AlphaHydrae.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    public static OI oi;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static driveTrainSubsystem driveTrainSubsystem;
    public static intakeSubsystem intakeSubsystem;
    public static clawSubsystem clawSubsystem;
    public static limelightSubsystem limelightSubsystem;
    public static elevatorPIDSubsystem elevatorPIDSubsystem;
    public static shiftingSubsystem shiftingSubsystem;
    public static rocketSubsystem rocketSubsystem;
    public static navXSub navXSub;
    public static climbingPIDSubsystem climbingPIDSubsystem;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveTrainSubsystem = new driveTrainSubsystem();
        intakeSubsystem = new intakeSubsystem();
        clawSubsystem = new clawSubsystem();
        limelightSubsystem = new limelightSubsystem();
        elevatorPIDSubsystem = new elevatorPIDSubsystem();
        shiftingSubsystem = new shiftingSubsystem();
        rocketSubsystem = new rocketSubsystem();
        navXSub = new navXSub();
        climbingPIDSubsystem = new climbingPIDSubsystem();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        SmartDashboard.putData("Auto mode", chooser);
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){

    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        //autonomousCommand = chooser.getSelected();
        // schedule the autonomous command (example)
        //if (autonomousCommand != null) autonomousCommand.start();

        Robot.elevatorPIDSubsystem.startupRoutine();
        Robot.intakeSubsystem.startingPosition();
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();


    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();

        Robot.intakeSubsystem.startingPosition();
        Robot.elevatorPIDSubsystem.zeroTheTalon();
        Robot.clawSubsystem.raiseClaw();
        Robot.elevatorPIDSubsystem.startupRoutine();
        Robot.climbingPIDSubsystem.startup();
        Robot.navXSub.resetNavX();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

        Robot.elevatorPIDSubsystem.homeEncoder();
        Robot.elevatorPIDSubsystem.followMaster();

        SmartDashboard.putNumber("motorOutput", Robot.elevatorPIDSubsystem.motorOutput());
        SmartDashboard.putNumber("SlaveOutput", Robot.elevatorPIDSubsystem.motorSlaveOutput());
        SmartDashboard.putString("Desired Prox", Robot.elevatorPIDSubsystem.returnDesiredProx());
        SmartDashboard.putBoolean("Low Prox", Robot.elevatorPIDSubsystem.getProx("Low"));
        SmartDashboard.putBoolean("Mid Prox", Robot.elevatorPIDSubsystem.getProx("Mid"));
        SmartDashboard.putBoolean("Max Prox", Robot.elevatorPIDSubsystem.getProx("Max"));
        SmartDashboard.putBoolean("atClimbingLevel", Robot.elevatorPIDSubsystem.returnAtClimbingSetpoint());
        SmartDashboard.putNumber("X", Robot.driveTrainSubsystem.returnLX());
        SmartDashboard.putNumber("Output Value", Robot.driveTrainSubsystem.forwardUsingArea());
        SmartDashboard.putNumber("Offset", Robot.driveTrainSubsystem.createOffset());
        //SmartDashboard.putNumber("Foot Enconder", Robot.climbingPIDSubsystem.footEncoderValue());
        SmartDashboard.putNumber("Elevator Enconder", Robot.elevatorPIDSubsystem.elevatorEncoderValue());
        SmartDashboard.putNumber("Pitch", Robot.navXSub.navXPitch());
        SmartDashboard.putNumber("Climbing Modifier", Robot.climbingPIDSubsystem.liftingModifier());
        SmartDashboard.putBoolean("Rocket Forward", Robot.oi.rocketPlacementForwardButtonValue());
        SmartDashboard.putNumber("CamMode", Robot.limelightSubsystem.returnCamMode());
        SmartDashboard.putNumber("Limelight Area", Robot.driveTrainSubsystem.returnLimelightArea());
        SmartDashboard.putBoolean("FootProx Value", Robot.climbingPIDSubsystem.atFootProx());
    }
}