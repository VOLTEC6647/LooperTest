/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc6647.robot;

import org.usfirst.frc6647.subsystems.Chassis;
import org.usfirst.lib6647.loops.LooperRobot;
import org.usfirst.lib6647.oi.JController;

public class Robot extends LooperRobot {

	/** Static {@link Robot} instance. */
	private static Robot instance;

	/**
	 * Gets running {@link Robot} instance.
	 * 
	 * @return instance
	 */
	public static Robot getInstance() {
		return instance;
	}

	protected Robot() {
		super(Chassis::new);
		instance = this;

		joysticks.put("driver1", new JController(0));
	}
}
