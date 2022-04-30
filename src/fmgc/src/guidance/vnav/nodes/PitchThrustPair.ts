class PitchThrustPairBuilder {
    private static atmosphericConditions: AtmosphericConditions;

    static atClimbThrust(): ConstantThrustPitchThrustPairBuilder {
        return new ClimbThrustPitchThrustPairBuilder(this.atmosphericConditions);
    }

    static atIdleThrust(): ConstantThrustPitchThrustPairBuilder {
        return new IdleThrustPitchThrustPairBuilder(this.atmosphericConditions);
    }

    static atSpeedThrust(): ConstantSpeedPitchThrustPairBuilder {
        return new SpeedPitchThrustPairBuilder(this.atmosphericConditions);
    }

    static atMachThrust(): ConstantSpeedPitchThrustPairBuilder {
        return new MachPitchThrustPairBuilder(this.atmosphericConditions);
    }
}

interface ConstantThrustPitchThrustPairBuilder {
    pitchForSpeed(): PitchThrustPair;
    pitchForAcceleration(): PitchThrustPair;
    pitchForDeceleration(): PitchThrustPair;
    pitchForMach(): PitchThrustPair;
    pitchForVs(): PitchThrustPair;
    pitchForFpa(): PitchThrustPair;
}
class ClimbThrustPitchThrustPairBuilder implements ConstantThrustPitchThrustPairBuilder {
    constructor(private atmosphericConditions: AtmosphericConditions) { }

    pitchForSpeed(): PitchThrustPair {

    }

    pitchForMach(): PitchThrustPair {

    }

    pitchForVs(): PitchThrustPair {

    }

    pitchForFpa(): PitchThrustPair {

    }

    pitchForAcceleration(): PitchThrustPair {

    }

    pitchForDeceleration(): PitchThrustPair {

    }
}
class IdleThrustPitchThrustPairBuilder {
    constructor(private atmosphericConditions: AtmosphericConditions) { }

    pitchForSpeed(): PitchThrustPair {

    }

    pitchForMach(): PitchThrustPair {

    }

    pitchForVs(): PitchThrustPair {

    }

    pitchForFpa(): PitchThrustPair {

    }

    pitchForAcceleration(): PitchThrustPair {

    }

    pitchForDeceleration(): PitchThrustPair {

    }
}

interface ConstantSpeedPitchThrustPairBuilder {
    atVs(verticalSpeed: FeetPerMinute): PitchThrustPair
    atFpa(flightPathAngle: Degrees): PitchThrustPair
}

class SpeedPitchThrustPairBuilder implements ConstantSpeedPitchThrustPairBuilder {
    atVs(verticalSpeed: FeetPerMinute): PitchThrustPair {

    }

    atFpa(flightPathAngle: Degrees): PitchThrustPair {

    }
}

class MachPitchThrustPairBuilder implements ConstantSpeedPitchThrustPairBuilder {
    atVs(): PitchThrustPair {

    }

    atFpa(): PitchThrustPair {

    }
}

interface PitchThrustPair {
    getFlightPathAngle(): Degrees;
    getVerticalSpeed(): FeetPerMinute;
    getFuelFlow(): number;
}

class DefaultPitchThrustPair implements PitchThrustPair {
    constructor(private thrustSetting: ThrustSetting) {}

    getFlightPathAngle(altitude: Feet, drag: number): number {

    }
}
