package org.usfirst.frc6647.subsystems;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc6647.robot.Robot;
import org.usfirst.lib6647.loops.ILooper;
import org.usfirst.lib6647.loops.Loop;
import org.usfirst.lib6647.loops.LoopType;
import org.usfirst.lib6647.oi.JController;
import org.usfirst.lib6647.subsystem.PIDSuperSubsystem;
import org.usfirst.lib6647.subsystem.supercomponents.SuperAHRS;
import org.usfirst.lib6647.subsystem.supercomponents.SuperTalon;
import org.usfirst.lib6647.subsystem.supercomponents.SuperVictor;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Chassis extends PIDSuperSubsystem implements SuperAHRS, SuperTalon, SuperVictor {

	public Chassis() {
		super("chassis");

		initAHRS(robotMap, getName());
		initTalons(robotMap, getName());
		initVictors(robotMap, getName());

		getVictor("backLeft").follow(getTalon("frontLeft"));
		getVictor("backRight").follow(getTalon("frontRight"));
	}

	@Override
	public void registerLoops(ILooper looper) {
		looper.register(new Loop() {
			private DifferentialDrive drive;
			private JController joystick;
			private int rightAxisX = 5, rightAxisY;
			private AHRS navX;

			@Override
			public void onStart(double timestamp) {
				synchronized (Chassis.this) {
					joystick = Robot.getInstance().getJoystick("driver1");
					rightAxisY = joystick.getName().equals("Wireless Controller") ? 2 : 4;
					drive = new DifferentialDrive(getTalon("frontLeft"), getTalon("frontRight"));
					navX = ahrsDevices.get("navX");

					navX.reset(); // Reset navX every time the Robot is enabled.
					setSetpoint("gyro", navX.getYaw()); // Set current direction as setpoint.
				}
			}

			@Override
			public void onLoop(double timestamp) {
				synchronized (Chassis.this) {
					setSetpoint("gyro",
							Math.abs(joystick.getRawAxis(rightAxisY)) > 0.15
									|| Math.abs(joystick.getRawAxis(rightAxisX)) > 0.15
											? Math.toDegrees(Math.atan2(joystick.getRawAxis(rightAxisY),
													joystick.getRawAxis(rightAxisX)))
											: navX.getYaw());
					double output = getPIDController("gyro").calculate(navX.getYaw());
					drive.arcadeDrive(joystick.getLeftAxis(), output, false);
				}
			}

			@Override
			public void onStop(double timestamp) {
				synchronized (Chassis.this) {
					System.out.println("Gyro stopped at: " + timestamp + ".");
				}
			}

			@Override
			public LoopType getType() {
				return LoopType.ENABLED;
			}
		});
	}
}