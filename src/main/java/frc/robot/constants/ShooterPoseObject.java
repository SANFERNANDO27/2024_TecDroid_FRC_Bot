package frc.robot.constants;

public class ShooterPoseObject<MinDistance, MaxDistance, RespectiveAngle> {

    private final MinDistance minDistance;
    private final MaxDistance maxDistance;
    private final RespectiveAngle respectiveAngle;

    public ShooterPoseObject(final MinDistance minDistance, final MaxDistance maxDistance, final RespectiveAngle respectiveAngle) {
        this.minDistance = minDistance;
        this.maxDistance = maxDistance;
        this.respectiveAngle = respectiveAngle;
    }

    public MinDistance getMinDistance() {
        return this.minDistance;
    }

    public MaxDistance getMaxDistance() {
        return this.maxDistance;
    }

    public RespectiveAngle getEstimatedAngle() {
        return this.respectiveAngle;
    }
}
