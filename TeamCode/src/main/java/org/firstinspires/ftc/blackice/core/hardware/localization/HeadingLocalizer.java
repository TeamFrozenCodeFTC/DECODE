package org.firstinspires.ftc.blackice.core.hardware.localization;

/**
 * A Localizer that only provides the current heading.
 * Used for tele-op when only the heading is needed for field-centric driving.
 */
public interface HeadingLocalizer extends Localizer {
    static UnsupportedOperationException unsupportedPositionalMethodException() {
        return new UnsupportedOperationException(
            "This heading localizer is used for tele-op when only the heading is needed for " +
                "field-centric driving. No positional data can be provided.");
    }
    
    @Override
    default double getX() {
        throw unsupportedPositionalMethodException();
    }
    
    @Override
    default double getY() {
        throw unsupportedPositionalMethodException();
    }
    
    @Override
    default double getFieldVelocityX() {
        throw unsupportedPositionalMethodException();
    }
    
    @Override
    default double getFieldVelocityY() {
        throw unsupportedPositionalMethodException();
    }
    
    @Override
    default double getAngularVelocity() {
        throw unsupportedPositionalMethodException();
    }
}
