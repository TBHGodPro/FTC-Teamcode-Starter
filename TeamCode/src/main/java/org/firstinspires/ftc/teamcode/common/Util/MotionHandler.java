package org.firstinspires.ftc.teamcode.common.Util;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.Util.Profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.Util.Profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.common.Util.Profile.ProfileState;

public class MotionHandler {
    private int current = 0;
    private int target = 0;
    private double activeTarget = 0;

    private ElapsedTime timer;

    private PIDCoefficients pidCoefficients;
    private ProfileConstraints profileConstraints;

    private PIDController pid;

    private AsymmetricMotionProfile profile;

    public MotionHandler() {
        setPID(0, 0, 0);
        setProfileConstraints(0, 0, 0);

        this.timer = new ElapsedTime();

        setTarget(0, 0, true);
    }

    public MotionHandler setCurrent(int current) {
        this.current = current;

        return this;
    }

    public int getCurrent() {
        return current;
    }

    public MotionHandler setPID(double p, double i, double d) {
        pidCoefficients = new PIDCoefficients(p, i, d);

        pid.setPID(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);

        return this;
    }

    public MotionHandler setProfileConstraints(int velo, int accel, int decel) {
        profileConstraints = new ProfileConstraints(velo, accel, decel);

        return this;
    }

    public MotionHandler setTarget(int current, int target) {
        return setTarget(current, target, false);
    }

    public MotionHandler setTarget(int current, int target, boolean force) {
        this.current = current;
        if (this.target == target && this.profile != null && !force) return this;

        this.target = target;
        this.activeTarget = (double) target;

        profile = new AsymmetricMotionProfile(current, target, profileConstraints);
        timer.reset();

        return this;
    }

    public int getTarget() {
        return target;
    }

    public double calc(int current) {
        return this.calculate(current);
    }

    public double calculate(int current) {
        this.current = current;

        if (profileConstraints.velo == 0) {
            activeTarget = target;
        } else {
            ProfileState state = profile.calculate(timer.time());
            activeTarget = state.x;
        }

        double power = 0;

        power += pid.calculate((double) current, target);

        //power += getFeedForward(current);

        power = Range.clip(power, -1, 1);

        return power;
    }
}
