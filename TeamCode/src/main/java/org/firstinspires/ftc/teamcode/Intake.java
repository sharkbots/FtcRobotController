package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.tools.Button;
import org.firstinspires.ftc.teamcode.tools.MotorActionManager;
import org.firstinspires.ftc.teamcode.tools.OverrideMotor;
import org.firstinspires.ftc.teamcode.tools.PixelsDetection;
import org.firstinspires.ftc.teamcode.tools.ServoActionManager;

@Config
public class Intake {
    public final PixelsDetection pixels;
    public final OverrideMotor intakeMotor;
    public final Servo intakeFlipper;
    private final Button handlerLeftTrigger, handlerLeftStick_Up, handlerLeftStick_Down;
    public static double intakeFlipperUp = 0.9, intakeFlipperPixel5 = 0.4825, intakeFlipperPixel4 = 0.4665, intakeFlipperPixel3 = 0.4515;
    private FlipperPosition flipperPosition;
    public MotorActionManager intakeMotorActionManager;
    public ServoActionManager intakeFlipperActionManger;

    public Intake(HardwareMap hardwareMap, Button handlerLeftTrigger, Button handlerLeftStick_Up, Button handlerLeftStick_Down) {
        // Motors
        intakeMotor = new OverrideMotor((DcMotorEx)hardwareMap.dcMotor.get("intakeMotor"));
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeFlipper = hardwareMap.servo.get("intakeFlipper");

        pixels = new PixelsDetection(hardwareMap);

        intakeMotorActionManager = new MotorActionManager(intakeMotor);
        intakeFlipperActionManger = new ServoActionManager(intakeFlipper);

        intakeFlipper.scaleRange(0, 1.0);

        this.handlerLeftTrigger = handlerLeftTrigger;
        this.handlerLeftStick_Up = handlerLeftStick_Up;
        this.handlerLeftStick_Down = handlerLeftStick_Down;
        this.flipperPosition = FlipperPosition.UP;
    }


    public void startIntakeMotorWithNoEncoder(double power) {
        intakeMotorActionManager.startMotorNoEncoder(power);
    }

    public void stopIntakeMotor() {
        intakeMotorActionManager.stopMotor();
    }

    public enum FlipperPosition {
        UP(intakeFlipperUp),
        PIXEL5(intakeFlipperPixel5),
        PIXEL4(intakeFlipperPixel4),
        PIXEL3(intakeFlipperPixel3);

        public final double value;

        FlipperPosition(double value) {
            this.value = value;
        }


        public FlipperPosition next() {
            // Check if this is the last enum constant
            if (this.ordinal() == values().length - 1) {
                return values()[0]; // Stay on this if it's the last
            } else {
                return values()[this.ordinal() + 1]; // Move to the next otherwise
            }
        }

        public FlipperPosition previous() {
            // Returns the previous enum constant, or this if this is the first
            if (this.ordinal() == 0) {
                return this;
            } else {
                return values()[this.ordinal() - 1];
            }
        }
    }

    public void setIntakeFlipperPosition(FlipperPosition flipperPosition){
        intakeFlipperActionManger.setServoPosition(flipperPosition.value);
    }

    public void update(Boolean notInOutTake){
        // Manages Reject mode on Roomba as an override of its current power and state
        pixels.update();
        if(handlerLeftTrigger.Pressed()) {
            intakeMotor.setOverridePower(-1);
        } else if (handlerLeftTrigger.Released()) {
            intakeMotor.cancelOverridePower();
        }
        if (notInOutTake) {
            if (handlerLeftStick_Down.Pressed()) {
                flipperPosition = flipperPosition.next();
            } else if (handlerLeftStick_Up.Pressed()) {
                flipperPosition = flipperPosition.previous();
            }
            intakeFlipper.setPosition(flipperPosition.value);
        }
    }

}