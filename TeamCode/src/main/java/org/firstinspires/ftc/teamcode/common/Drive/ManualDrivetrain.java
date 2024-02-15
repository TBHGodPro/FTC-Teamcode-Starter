package org.firstinspires.ftc.teamcode.common.Drive;

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

    public final XIMU imu;

    public final XMotor frontLeft;
    public final XMotor frontRight;
    public final XMotor backLeft;
    public final XMotor backRight;

    public ManualDrivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.imu = new XIMU(
                hardwareMap.get(IMU.class, "imu"),
                DynamicConstants.imuParams,
                AngleUnit.DEGREES
        );

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
    public void setPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        this.frontLeft.setPower(frontLeft);
        this.frontRight.setPower(frontRight);
        this.backLeft.setPower(backLeft);
        this.backRight.setPower(backRight);
    }

    @Override
    public Double getDirection() {
        return imu.getYaw();
    }
}
