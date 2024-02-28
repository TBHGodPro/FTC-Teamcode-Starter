package org.firstinspires.ftc.teamcode.common.Wrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

public class XGamepad extends Gamepad {
    public final Gamepad real;

    public Gamepad last;

    public XGamepad(Gamepad gamepad) {
        this.real = gamepad;

        this.last = new Gamepad();

        update();
    }

    public XGamepad update() {
        last.copy(this);
        this.copy(real);

        return this;
    }
}
