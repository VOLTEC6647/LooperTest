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
import edu.wpi.first.wpiutil.math.MathUtil;

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
			DifferentialDrive drive;
			JController joystick;
			AHRS navX;

			@Override
			public void onStart(double timestamp) {
				synchronized (Chassis.this) {
					System.out.println("Gyro started at: " + timestamp + "!");

					joystick = Robot.getInstance().getJoystick("driver1");
					drive = new DifferentialDrive(getTalon("frontLeft"), getTalon("frontRight"));
					navX = ahrsDevices.get("navX");

					navX.reset();
					setSetpoint("gyro", navX.getYaw());
				}
			}

			@Override
			public void onLoop(double timestamp) {
				synchronized (Chassis.this) {
					setSetpoint("gyro",
							Math.abs(joystick.getRawAxis(4)) > 0.15 || Math.abs(joystick.getRawAxis(5)) > 0.15
									? Math.toDegrees(Math.atan2(joystick.getRawAxis(4), joystick.getRawAxis(5)))
									: navX.getYaw());

					double output = MathUtil
							.clamp(getPIDController("gyro").calculate(navX.getYaw(), getSetpoint("gyro")), -1.0, 1.0);
					drive.arcadeDrive(joystick.getRawAxis(1), output, false);
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