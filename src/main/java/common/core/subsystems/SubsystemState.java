package common.core.subsystems;

public interface SubsystemState<T extends Enum<T>> {
    
    public void setState(T state);

    public T getState();

    public boolean stateIs(T state);
}
