package frc.common3128.hardware.limelight;

public enum Pipeline {
    RED(0),
    BLUE(1);

    private int pipeline;
    private Pipeline(int pipeline) {
        this.pipeline = pipeline;
    }

    public int getPipeline() {
        return pipeline;
    }
}