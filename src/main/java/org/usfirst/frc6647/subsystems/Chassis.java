package org.usfirst.frc6647.subsystems;

import org.usfirst.frc6647.robot.Robot;
import org.usfirst.lib6647.loops.ILooper;
import org.usfirst.lib6647.loops.Loop;
import org.usfirst.lib6647.oi.JController;
import org.usfirst.lib6647.subsystem.PIDSuperSubsystem;
import org.usfirst.lib6647.subsystem.supercomponents.SuperAHRS;
import org.usfirst.lib6647.subsystem.supercomponents.SuperTalon;
import org.usfirst.lib6647.subsystem.supercomponents.SuperVictor;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Chassis extends PIDSuperSubsystem implements SuperAHRS, SuperTalon, SuperVictor {
	DifferentialDrive drive;
	JController joystick;

	public Chassis() {
		super("chassis");

		initAHRS(robotMap, getName());
		initTalons(robotMap, getName());
		initVictors(robotMap, getName());

		getVictor("backLeft").follow(getTalon("frontLeft"));
		getVictor("backRight").follow(getTalon("frontRight"));

		drive = new DifferentialDrive(getTalon("frontLeft"), getTalon("frontRight"));
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				System.out.println("Gyro started at: " + timestamp + "!");
				joystick = Robot.getInstance().getJoystick("driver1");
				setSetpoint(getPosition());
			}

			@Override
			public void onLoop(double timestamp) {
				if (Math.abs(joystick.getRawAxis(4)) > 0.15 || Math.abs(joystick.getRawAxis(5)) > 0.15)
					setSetpoint(Math.atan2(joystick.getRawAxis(4), joystick.getRawAxis(5)));

				double output = getPIDController().calculate(getPosition(), getSetpoint());
				drive.arcadeDrive(joystick.getRawAxis(1), output, false);
			}

			@Override
			public void onStop(double timestamp) {
				System.out.println("Gyro stopped at: " + timestamp + ".");
			}
		});
	}

	@Override
	public double getPosition() {
		return ahrsDevices.get("navX").getYaw();
	}
}