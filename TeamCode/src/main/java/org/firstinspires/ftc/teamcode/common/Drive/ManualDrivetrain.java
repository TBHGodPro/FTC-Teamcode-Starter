package org.firstinspires.ftc.teamcode.common.Drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Util.MotionHandler;
import org.firstinspires.ftc.teamcode.common.Util.Side;
import org.firstinspires.ftc.teamcode.common.Util.Wrappers.XIMU;
import org.firstinspires.ftc.teamcode.common.Util.Wrappers.XMotor;
import org.firstinspires.ftc.teamcode.custom.DynamicConstants;

public class ManualDrivetrain implements BaseDrivetrain {
    public final HardwareMap hardwareMap;

    // IMU
    public final XIMU imu;
    public final PIDController headingPID;

    // WHEELS
    public final XMotor frontLeft;
    public final XMotor frontRight;
    public final XMotor backLeft;
    public final XMotor backRight;

    // DYNAMIC
    public Double currentHeading = null;
    private boolean wasTurning = false;

    public ManualDrivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.imu = new XIMU(
                hardwareMap.get(IMU.class, "imu"),
                DynamicConstants.imuParams,
                AngleUnit.DEGREES
        );

        this.headingPID = new PIDController(DynamicConstants.headingPID.p, DynamicConstants.headingPID.i, DynamicConstants.headingPID.d);

        this.frontLeft = new XMotor(hardwareMap, DynamicConstants.frontLeftName, new MotionHandler())
                .setInverted(DynamicConstants.wheelInvertedSide == Side.LEFT)
                .setZeroPowerBehavior(DynamicConstants.wheelZeroPowerBehavior)
                .setShouldHandleVoltage(true);
        this.frontRight = new XMotor(hardwareMap, DynamicConstants.frontRightName, new MotionHandler())
                .setInverted(DynamicConstants.wheelInvertedSide == Side.RIGHT)
                .setZeroPowerBehavior(DynamicConstants.wheelZeroPowerBehavior)
                .setShouldHandleVoltage(true);
        this.backLeft = new XMotor(hardwareMap, DynamicConstants.backLeftName, new MotionHandler())
                .setInverted(DynamicConstants.wheelInvertedSide == Side.LEFT)
                .setZeroPowerBehavior(DynamicConstants.wheelZeroPowerBehavior)
                .setShouldHandleVoltage(true);
        this.backRight = new XMotor(hardwareMap, DynamicConstants.backRightName, new MotionHandler())
                .setInverted(DynamicConstants.wheelInvertedSide == Side.RIGHT)
                .setZeroPowerBehavior(DynamicConstants.wheelZeroPowerBehavior)
                .setShouldHandleVoltage(true);
    }

    @Override
    public void setWheelPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        this.frontLeft.setPower(frontLeft);
        this.frontRight.setPower(frontRight);
        this.backLeft.setPower(backLeft);
        this.backRight.setPower(backRight);
    }

    @Override
    public Double getHeading() {
        return imu.getYaw();
    }

    public void update(double forward, double strafe, double turn, boolean fieldCentric) {
        // Save Heading
        double heading = getHeading();

        // Save Inputs
        double controlForward = forward;
        double controlStrafe = strafe;
        double controlTurn = turn;

        // Turning Non-Linearity
        controlTurn = controlTurn >= 0 ? Math.pow(controlTurn, DynamicConstants.turningSlope) : -Math.pow(-controlTurn, DynamicConstants.turningSlope);

        // Heading PID
        if (wasTurning) {
            if (controlTurn == 0 && Math.abs(imu.getVelocityDegrees().zRotationRate) < 20)
                wasTurning = false;
        } else {
            if (controlTurn != 0) {
                currentHeading = null;
                wasTurning = true;
            } else {
                if (currentHeading == null) currentHeading = heading;
                else {
                    controlTurn += getHeadingCorrectionPower(heading) * 0.75;
                }
            }
        }

        // Field-Centric Driving
        if (fieldCentric) {
            double angleDiff = (0 - heading) * Math.PI / 180;

            double cos = Math.cos(angleDiff);
            double sin = Math.sin(angleDiff);

            controlStrafe = cos * controlStrafe - sin * controlForward;
            controlForward = sin * controlStrafe + cos * controlForward;
        }

        // Add Up Powers
        double frontLeft = controlForward + controlStrafe + controlTurn;
        double frontRight = controlForward - controlStrafe - controlTurn;
        double backLeft = controlForward - controlStrafe + controlTurn;
        double backRight = controlForward + controlStrafe - controlTurn;

        // Send Powers
        setWheelPowers(frontLeft, frontRight, backLeft, backRight);
    }

    public double getHeadingCorrectionPower(double heading) {
        headingPID.setPID(DynamicConstants.headingPID.p, DynamicConstants.headingPID.i, DynamicConstants.headingPID.d);

        double diff = currentHeading - heading;

        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;

        return headingPID.calculate(diff, 0);
    }

    public void resetHeading() {
        imu.resetYaw();
        currentHeading = 0d;
    }
}
