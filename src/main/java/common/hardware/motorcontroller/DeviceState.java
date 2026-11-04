package common.hardware.motorcontroller;

/** The health of a motor controller, from {@link NAR_Motor#getState()}, for example, for a dashboard indicator. */
public enum DeviceState {
    /** Connected, with no faults. */
    RUNNING,
    /** Connected, but reporting a fault. */
    PARTIALLY_RUNNING,
    /** Not reporting, or reporting an error. */
    DISCONNECTED
}
