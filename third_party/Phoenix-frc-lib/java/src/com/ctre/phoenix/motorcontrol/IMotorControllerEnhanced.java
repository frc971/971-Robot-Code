package com.ctre.phoenix.motorcontrol;

import com.ctre.phoenix.ErrorCode;

public interface IMotorControllerEnhanced extends IMotorController {
	 //------ Set output routines. ----------//
    /* in parent */

    //------ Invert behavior ----------//
    /* in parent */

    //----- general output shaping ------------------//
    /* in parent */

    //------ Voltage Compensation ----------//
    /* in parent */

    //------ General Status ----------//
    /* in parent */

    //------ sensor selection ----------//
    /* expand the options */
	 public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs );

    //------- sensor status --------- //
    /* in parent */

    //------ status frame period changes ----------//
    public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs );
    public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs );

    //----- velocity signal conditionaing ------//
    public ErrorCode configVelocityMeasurementPeriod(VelocityMeasPeriod period, int timeoutMs );
    public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs );

    //------ remote limit switch ----------//
    /* in parent */

    //------ local limit switch ----------//
    public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int timeoutMs );
    public ErrorCode configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int timeoutMs );

    //------ soft limit ----------//
    /* in parent */

    //------ Current Lim ----------//
    public ErrorCode configPeakCurrentLimit(int amps, int timeoutMs );
    public ErrorCode configPeakCurrentDuration(int milliseconds, int timeoutMs );
    public ErrorCode configContinuousCurrentLimit(int amps, int timeoutMs );
    public void enableCurrentLimit(boolean enable);

    //------ General Close loop ----------//
    /* in parent */

    //------ Motion Profile Settings used in Motion Magic and Motion Profile ----------//
    /* in parent */

    //------ Motion Profile Buffer ----------//
    /* in parent */

    //------ error ----------//
    /* in parent */

    //------ Faults ----------//
    /* in parent */

    //------ Firmware ----------//
    /* in parent */

    //------ Custom Persistent Params ----------//
    /* in parent */

    //------ Generic Param API, typically not used ----------//
    /* in parent */

    //------ Misc. ----------//
    /* in parent */

    //------ RAW Sensor API ----------//
    /**
     * @retrieve object that can get/set individual RAW sensor values.
     */
    //SensorCollection SensorCollection { get; }
    
    
}