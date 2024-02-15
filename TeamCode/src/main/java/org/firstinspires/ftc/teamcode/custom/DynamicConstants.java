package org.firstinspires.ftc.teamcode.custom;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.common.Util.Side;

@Config
public class DynamicConstants {
    // IMU
    public static final IMU.Parameters imuParams = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
    );

    // WHEELS
    public static final String frontLeftName = "front_left";
    public static final String frontRightName = "front_right";
    public static final String backLeftName = "back_left";
    public static final String backRightName = "back_right";
    public static final Side wheelInvertedSide = Side.LEFT;
    public static final DcMotor.ZeroPowerBehavior wheelZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
}
