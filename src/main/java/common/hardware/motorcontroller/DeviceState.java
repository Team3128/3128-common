package common.hardware.motorcontroller;

/** How a motor controller is doing, from {@link NAR_Motor#getState()}, e.g. for a light on the dashboard. */
public enum DeviceState {
    /** Connected, with no faults. */
    RUNNING,
    /** Connected, but reporting a fault. */
    PARTIALLY_RUNNING,
    /** Not reporting, or reporting an error. */
    DISCONNECTED
}
