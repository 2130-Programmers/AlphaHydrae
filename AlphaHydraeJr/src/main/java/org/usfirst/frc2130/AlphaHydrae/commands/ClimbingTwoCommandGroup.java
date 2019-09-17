/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc2130.AlphaHydrae.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbingTwoCommandGroup extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ClimbingTwoCommandGroup() {
    
    //addSequential(new LiftElevatorLevelThree());
    addSequential(new LiftElevatorLevelTwo());
    addSequential(new AutonomousCommand(), 1);
    addSequential(new LiftRobot());
    addSequential(new MoveRobot());
    addSequential(new RetractFoot());

  }
}
