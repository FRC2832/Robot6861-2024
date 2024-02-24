package org.livoniawarriors;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * This class is a joystick wrapper for the Logitech Heavy Equipment Side Panel
 * https://www.logitechg.com/en-us/products/farm/farm-simulator-side-panel.945-000031.html
 */
public class Saitek extends GenericHID {

    /** Represents a digital button on an Saitek. */
    public enum Button {
        PINK_TOP_LEFT(1),
        PINK_TOP_MIDDLE(2),
        PINK_TOP_RIGHT(3),
        PINK_BOTTOM_LEFT(6),
        PINK_BOTTOM_MIDDLE(7),
        PINK_BOTTOM_RIGHT(8),
        ORANGE_TOP_LEFT(4),
        ORANGE_TOP_RIGHT(5),
        ORANGE_BOTTOM_LEFT(9),
        ORANGE_BOTTOM_RIGHT(10),
        YELLOW_TOP_LEFT(11),
        YELLOW_TOP_MIDDLE(13),
        YELLOW_TOP_RIGHT(15),
        YELLOW_BOTTOM_LEFT(12),
        YELLOW_BOTTOM_MIDDLE(14),
        YELLOW_BOTTOM_RIGHT(16);

        public final int value;

        Button(int value) {
            this.value = value;
        }

    }

    public enum Axis {
        X_AXIS_1(0),
        Y_AXIS_1(1),
        ROTATE_1(2),
        X_AXIS_2(3),
        Y_AXIS_2(4),
        ROTATE_2(5);

        public final int value;

        Axis(int value) {
            this.value = value;
        }

    }

    public Saitek(final int port) {
        super(port);
    }

    public double getxAxis1() {
        return getRawAxis(Axis.X_AXIS_1.value);
    }

    public double getyAxis1() {
        return getRawAxis(Axis.Y_AXIS_1.value);
    }

    public double rotate1() {
        return getRawAxis(Axis.ROTATE_1.value);
    }

    public double getxAxis2() {
        return getRawAxis(Axis.X_AXIS_2.value);
    }

    public double getyAxis2() {
        return getRawAxis(Axis.Y_AXIS_2.value);
    }

    public double rotate2() {
        return getRawAxis(Axis.ROTATE_2.value);
    }

    // Pink Buttons
    // Top Left
    public boolean getPinkTopLeftButton() {
        return getRawButton(Button.PINK_TOP_LEFT.value);
    }

    public boolean getPinkTopLeftButtonPressed() {
        return getRawButtonPressed(Button.PINK_TOP_LEFT.value);
    }

    public boolean getPinkTopLeftButtonReleased() {
        return getRawButtonReleased(Button.PINK_TOP_LEFT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent pinkTopLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getPinkTopLeftButton);
    }

    // Top middle
    public boolean getPinkTopMiddleButton() {
        return getRawButton(Button.PINK_TOP_MIDDLE.value);
    }

    public boolean getPinkTopMiddleButtonPressed() {
        return getRawButtonPressed(Button.PINK_TOP_MIDDLE.value);
    }

    public boolean getPinkTopMiddleButtonReleased() {
        return getRawButtonReleased(Button.PINK_TOP_MIDDLE.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent pinkTopMiddle(EventLoop loop) {
        return new BooleanEvent(loop, this::getPinkTopMiddleButton);
    }

    // Top Right
    public boolean getPinkTopRightButton() {
        return getRawButton(Button.PINK_TOP_RIGHT.value);
    }

    public boolean getPinkTopRightButtonPressed() {
        return getRawButtonPressed(Button.PINK_TOP_RIGHT.value);
    }

    public boolean getPinkTopRightButtonReleased() {
        return getRawButtonReleased(Button.PINK_TOP_RIGHT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent pinkTopRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getPinkTopRightButton);
    }

    // Bottom Left
    public boolean getPinkBottomLeftButton() {
        return getRawButton(Button.PINK_BOTTOM_LEFT.value);
    }

    public boolean getPinkBottomLeftButtonPressed() {
        return getRawButtonPressed(Button.PINK_BOTTOM_LEFT.value);
    }

    public boolean getPinkBottomLeftButtonReleased() {
        return getRawButtonReleased(Button.PINK_BOTTOM_LEFT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent pinkBottomLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getPinkBottomLeftButton);
    }

    // Bottom Middle
    public boolean getPinkBottomMiddleButton() {
        return getRawButton(Button.PINK_BOTTOM_MIDDLE.value);
    }

    public boolean getPinkBottomMiddleButtonPressed() {
        return getRawButtonPressed(Button.PINK_BOTTOM_MIDDLE.value);
    }

    public boolean getPinkBottomMiddleButtonReleased() {
        return getRawButtonReleased(Button.PINK_BOTTOM_MIDDLE.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent pinkBottomMiddle(EventLoop loop) {
        return new BooleanEvent(loop, this::getPinkBottomMiddleButton);
    }

    // Bottom Right
    public boolean getPinkBottomRightButton() {
        return getRawButton(Button.PINK_BOTTOM_RIGHT.value);
    }

    public boolean getPinkBottomRightButtonPressed() {
        return getRawButtonPressed(Button.PINK_BOTTOM_RIGHT.value);
    }

    public boolean getPinkBottomRightButtonReleased() {
        return getRawButtonReleased(Button.PINK_BOTTOM_RIGHT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent pinkBottomRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getPinkBottomRightButton);
    }

    // Orange Buttons
    // Top Left
    public boolean getOrangeTopLeftButton() {
        return getRawButton(Button.ORANGE_TOP_LEFT.value);
    }

    public boolean getOrangeTopLeftButtonPressed() {
        return getRawButtonPressed(Button.ORANGE_TOP_LEFT.value);
    }

    public boolean getOrangeTopLeftButtonReleased() {
        return getRawButtonReleased(Button.ORANGE_TOP_LEFT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent orangeTopLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getOrangeTopLeftButton);
    }

    // Top Right
    public boolean getOrangeTopRightButton() {
        return getRawButton(Button.ORANGE_TOP_RIGHT.value);
    }

    public boolean getOrangeTopRightButtonPressed() {
        return getRawButtonPressed(Button.ORANGE_TOP_RIGHT.value);
    }

    public boolean getOrangeTopRightButtonReleased() {
        return getRawButtonReleased(Button.ORANGE_TOP_RIGHT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent orangeTopRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getOrangeTopRightButton);
    }

    // Bottom Left
    public boolean getOrangeBottomLeftButton() {
        return getRawButton(Button.ORANGE_BOTTOM_LEFT.value);
    }

    public boolean getOrangeBottomLeftButtonPressed() {
        return getRawButtonPressed(Button.ORANGE_BOTTOM_LEFT.value);
    }

    public boolean getOrangeBottomLeftButtonReleased() {
        return getRawButtonReleased(Button.ORANGE_BOTTOM_LEFT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent orangeBottomLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getOrangeBottomLeftButton);
    }

    // Bottom Right
    public boolean getOrangeBottomRightButton() {
        return getRawButton(Button.ORANGE_BOTTOM_RIGHT.value);
    }

    public boolean getOrangeBottomRightButtonPressed() {
        return getRawButtonPressed(Button.ORANGE_BOTTOM_RIGHT.value);
    }

    public boolean getOrangeBottomRightButtonReleased() {
        return getRawButtonReleased(Button.ORANGE_BOTTOM_RIGHT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent orangeBottomRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getOrangeBottomRightButton);
    }

    // Yellow Buttons
    // Top Left
    public boolean getYellowTopLeftButton() {
        return getRawButton(Button.YELLOW_TOP_LEFT.value);
    }

    public boolean getYellowTopLeftButtonPressed() {
        return getRawButtonPressed(Button.YELLOW_TOP_LEFT.value);
    }

    public boolean getYellowTopLeftButtonReleased() {
        return getRawButtonReleased(Button.YELLOW_TOP_LEFT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent yellowTopLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getYellowTopLeftButton);
    }

    // Top Middle
    public boolean getYellowTopMiddleButton() {
        return getRawButton(Button.YELLOW_TOP_MIDDLE.value);
    }

    public boolean getYellowTopMiddleButtonPressed() {
        return getRawButtonPressed(Button.YELLOW_TOP_MIDDLE.value);
    }

    public boolean getYellowTopMiddleButtonReleased() {
        return getRawButtonReleased(Button.YELLOW_TOP_MIDDLE.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent yellowTopMiddle(EventLoop loop) {
        return new BooleanEvent(loop, this::getYellowTopMiddleButton);
    }

    // Top Right
    public boolean getYellowTopRightButton() {
        return getRawButton(Button.YELLOW_TOP_RIGHT.value);
    }

    public boolean getYellowTopRightButtonPressed() {
        return getRawButtonPressed(Button.YELLOW_TOP_RIGHT.value);
    }

    public boolean getYellowTopRightButtonReleased() {
        return getRawButtonReleased(Button.YELLOW_TOP_RIGHT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent yellowTopRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getYellowTopRightButton);
    }

    // Bottom Left
    public boolean getYellowBottomLeftButton() {
        return getRawButton(Button.YELLOW_BOTTOM_LEFT.value);
    }

    public boolean getYellowBottomLeftButtonPressed() {
        return getRawButtonPressed(Button.YELLOW_BOTTOM_LEFT.value);
    }

    public boolean getYellowBottomLeftButtonReleased() {
        return getRawButtonReleased(Button.YELLOW_BOTTOM_LEFT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent yellowBottomLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getYellowBottomLeftButton);
    }

    // Bottom Middle
    public boolean getYellowBottomMiddleButton() {
        return getRawButton(Button.YELLOW_BOTTOM_LEFT.value);
    }

    public boolean getYellowBottomMiddleButtonPressed() {
        return getRawButtonPressed(Button.YELLOW_BOTTOM_MIDDLE.value);
    }

    public boolean getYellowBottomMiddleButtonReleased() {
        return getRawButtonReleased(Button.YELLOW_BOTTOM_MIDDLE.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent yellowBottomMiddle(EventLoop loop) {
        return new BooleanEvent(loop, this::getYellowBottomMiddleButton);
    }

    // Bottom Right
    public boolean getYellowBottomRightButton() {
        return getRawButton(Button.YELLOW_BOTTOM_RIGHT.value);
    }

    public boolean getYellowBottomRightButtonPressed() {
        return getRawButtonPressed(Button.YELLOW_BOTTOM_RIGHT.value);
    }

    public boolean getYellowBottomRightButtonReleased() {
        return getRawButtonReleased(Button.YELLOW_BOTTOM_RIGHT.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent yellowBottomRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getYellowBottomRightButton);
    }
}