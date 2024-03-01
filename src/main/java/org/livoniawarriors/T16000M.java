package org.livoniawarriors;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * This class is a joystick wrapper for the ThrustMaster T16000M joystick
 * https://www.thrustmaster.com/products/t-16000m-fcs/
 */
public class T16000M extends GenericHID {
    /** Represents a digital button on an Saitek. */
    public enum Button {
        TRIGGER(1),
        MIDDLE(2),
        LEFT(3),
        RIGHT(4),
        LEFT_SIDE_UP_LEFT(5),
        LEFT_SIDE_UP_MIDDLE(6),
        LEFT_SIDE_UP_RIGHT(7),
        LEFT_SIDE_DOWN_RIGHT(8),
        LEFT_SIDE_DOWN_MIDDLE(9),
        LEFT_SIDE_DOWN_LEFT(10),
        RIGHT_SIDE_UP_LEFT(11),
        RIGHT_SIDE_UP_MIDDLE(12),
        RIGHT_SIDE_UP_RIGHT(13),
        RIGHT_SIDE_DOWN_RIGHT(14),
        RIGHT_SIDE_DOWN_MIDDLE(15),
        RIGHT_SIDE_DOWN_LEFT(16);

        public final int value;

        Button(int value) {
            this.value = value;
        }

    }

    public enum Axis {
        X_AXIS(0),
        Y_AXIS(1),
        ROTATE(2),
        SLIDER(3);

        public final int value;

        Axis(int value) {
            this.value = value;
        }

    }

    public T16000M(final int port) {
        super(port);
    }

    public double getxAxis1() {
        return getRawAxis(Axis.X_AXIS.value);
    }

    public double getyAxis1() {
        return getRawAxis(Axis.Y_AXIS.value);
    }

    public double rotate() {
        return getRawAxis(Axis.ROTATE.value);
    }

    public double slider() {
        return getRawAxis(Axis.SLIDER.value);
    }

    // Trigger Detection
    public boolean getTrigger() {
        return getRawButton(Button.TRIGGER.value);
    }

    public boolean getTriggerPressed() {
        return getRawButtonPressed(Button.TRIGGER.value);
    }

    public boolean getTriggerReleased() {
        return getRawButtonReleased(Button.TRIGGER.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent trigger(EventLoop loop) {
        return new BooleanEvent(loop, this::getTrigger);
    }

    public boolean getMiddle() {
        return getRawButton(Button.MIDDLE.value);
    }

    public boolean getMiddlePressed() {
        return getRawButtonPressed(Button.MIDDLE.value);
    }

    public boolean getMiddleReleased() {
        return getRawButtonReleased(Button.MIDDLE.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent middle(EventLoop loop) {
        return new BooleanEvent(loop, this::getMiddle);
    }

    public boolean getLeft() {
        return getRawButton(Button.LEFT.value);
    }

    public boolean getLeftPressed() {
        return getRawButtonPressed(Button.LEFT.value);
    }

    public boolean getLeftReleased() {
        return getRawButtonReleased(Button.LEFT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent left(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeft);
    }

    public boolean getRight() {
        return getRawButton(Button.MIDDLE.value);
    }

    public boolean getRightPressed() {
        return getRawButtonPressed(Button.RIGHT.value);
    }

    public boolean getRightReleased() {
        return getRawButtonReleased(Button.RIGHT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent right(EventLoop loop) {
        return new BooleanEvent(loop, this::getRight);
    }

    public boolean getLeftSideUpLeft() {
        return getRawButton(Button.LEFT_SIDE_UP_LEFT.value);
    }

    public boolean getLeftSideUpLeftPressed() {
        return getRawButtonPressed(Button.LEFT_SIDE_UP_LEFT.value);
    }

    public boolean getLeftSideUpLeftReleased() {
        return getRawButtonReleased(Button.LEFT_SIDE_UP_LEFT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent leftSideUpLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftSideUpLeft);
    }

    public boolean getLeftSideUpMiddle() {
        return getRawButton(Button.LEFT_SIDE_UP_MIDDLE.value);
    }

    public boolean getLeftSideUpMiddlePressed() {
        return getRawButtonPressed(Button.LEFT_SIDE_UP_MIDDLE.value);
    }

    public boolean getLeftSideUpMiddleReleased() {
        return getRawButtonReleased(Button.LEFT_SIDE_UP_MIDDLE.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent leftSideUpMiddle(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftSideUpMiddle);
    }

    public boolean getLeftSideUpRight() {
        return getRawButton(Button.LEFT_SIDE_UP_RIGHT.value);
    }

    public boolean getLeftSideUpRightPressed() {
        return getRawButtonPressed(Button.LEFT_SIDE_UP_RIGHT.value);
    }

    public boolean getLeftSideUpRightReleased() {
        return getRawButtonReleased(Button.LEFT_SIDE_UP_MIDDLE.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent leftSideUpRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftSideUpRight);
    }

    public boolean getLeftSideDownRight() {
        return getRawButton(Button.LEFT_SIDE_DOWN_RIGHT.value);
    }

    public boolean getLeftSideDownRightPressed() {
        return getRawButtonPressed(Button.LEFT_SIDE_DOWN_RIGHT.value);
    }

    public boolean getLeftSideDownRightReleased() {
        return getRawButtonReleased(Button.LEFT_SIDE_DOWN_RIGHT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent leftSideDownRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftSideDownRight);
    }

    public boolean getLeftSideDownMiddle() {
        return getRawButton(Button.LEFT_SIDE_DOWN_MIDDLE.value);
    }

    public boolean getLeftSideDownMiddlePressed() {
        return getRawButtonPressed(Button.LEFT_SIDE_DOWN_MIDDLE.value);
    }

    public boolean getLeftSideDownMiddleReleased() {
        return getRawButtonReleased(Button.LEFT_SIDE_DOWN_MIDDLE.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent leftSideDownMiddle(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftSideDownMiddle);
    }

    public boolean getLeftSideDownLeft() {
        return getRawButton(Button.LEFT_SIDE_DOWN_LEFT.value);
    }

    public boolean getLeftSideDownLeftPressed() {
        return getRawButtonPressed(Button.LEFT_SIDE_DOWN_LEFT.value);
    }

    public boolean getLeftSideDownLeftReleased() {
        return getRawButtonReleased(Button.LEFT_SIDE_DOWN_LEFT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent leftSideDownLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftSideDownLeft);
    }

    public boolean getRightSideUpLeft() {
        return getRawButton(Button.RIGHT_SIDE_UP_LEFT.value);
    }

    public boolean getRightSideUpLeftPressed() {
        return getRawButtonPressed(Button.RIGHT_SIDE_UP_LEFT.value);
    }

    public boolean getRightSideUpLeftReleased() {
        return getRawButtonReleased(Button.LEFT_SIDE_UP_LEFT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent rightSideUpLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightSideUpLeft);
    }

    public boolean getRightSideUpMiddle() {
        return getRawButton(Button.LEFT_SIDE_UP_MIDDLE.value);
    }

    public boolean getRightSideUpMiddlePressed() {
        return getRawButtonPressed(Button.RIGHT_SIDE_UP_MIDDLE.value);
    }

    public boolean getRightSideUpMiddleReleased() {
        return getRawButtonReleased(Button.RIGHT_SIDE_UP_MIDDLE.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent rightSideUpMiddle(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightSideUpMiddle);
    }

    public boolean getRightSideUpRight() {
        return getRawButton(Button.RIGHT_SIDE_UP_RIGHT.value);
    }

    public boolean getRightSideUpRightPressed() {
        return getRawButtonPressed(Button.RIGHT_SIDE_UP_RIGHT.value);
    }

    public boolean getRightSideUpRightReleased() {
        return getRawButtonReleased(Button.RIGHT_SIDE_UP_MIDDLE.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent rightSideUpRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightSideUpRight);
    }

    public boolean getRightSideDownRight() {
        return getRawButton(Button.RIGHT_SIDE_DOWN_RIGHT.value);
    }

    public boolean getRightSideDownRightPressed() {
        return getRawButtonPressed(Button.RIGHT_SIDE_DOWN_RIGHT.value);
    }

    public boolean getRightSideDownRightReleased() {
        return getRawButtonReleased(Button.RIGHT_SIDE_DOWN_RIGHT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent rightSideDownRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightSideDownRight);
    }

    public boolean getRightSideDownMiddle() {
        return getRawButton(Button.RIGHT_SIDE_DOWN_MIDDLE.value);
    }

    public boolean getRightSideDownMiddlePressed() {
        return getRawButtonPressed(Button.RIGHT_SIDE_DOWN_MIDDLE.value);
    }

    public boolean getRightSideDownMiddleReleased() {
        return getRawButtonReleased(Button.RIGHT_SIDE_DOWN_MIDDLE.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent rightSideDownMiddle(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightSideDownMiddle);
    }

    public boolean getRightSideDownLeft() {
        return getRawButton(Button.RIGHT_SIDE_DOWN_LEFT.value);
    }

    public boolean getRightSideDownLeftPressed() {
        return getRawButtonPressed(Button.RIGHT_SIDE_DOWN_LEFT.value);
    }

    public boolean getRightSideDownLeftReleased() {
        return getRawButtonReleased(Button.RIGHT_SIDE_DOWN_LEFT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent rightSideDownLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightSideDownLeft);
    }
}