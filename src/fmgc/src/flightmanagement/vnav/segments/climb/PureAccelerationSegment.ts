import { IntegrationEndConditions, IntegrationPropagator, Integrator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { AccelFactorMode } from '@fmgc/guidance/vnav/common';

export class PureAccelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

    private useMachTarget: boolean = false;

    private toMach: Mach = 0.82;

    constructor(
        private accelerationPropagator: IntegrationPropagator,
        private toSpeed: Knots,
        toAltitude: Feet,
        maxDistance: NauticalMiles = Infinity,
        toMach: Mach = 0.82,
    ) {
        super();
        this.toMach = toMach;

        this.endConditions = {
            calibratedAirspeed: { max: toSpeed },
            mach: { max: toMach },
            altitude: { max: toAltitude },
            distanceFromStart: { max: maxDistance },
        };
    }

    /**
     * A helper function to initialize a PureAccelerationSegment to accelerate to a mach target
     */
    static toMach(
        accelerationPropagator: IntegrationPropagator,
        toSpeed: Knots,
        toMach: Mach,
        toAltitude: Feet,
        maxDistance: NauticalMiles = Infinity,
    ): PureAccelerationSegment {
        const segment = new PureAccelerationSegment(
            accelerationPropagator,
            toSpeed,
            toAltitude,
            maxDistance,
            toMach,
        );

        segment.useMachTarget = true;
        return segment;
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const step = this.integrator.integrate(state, this.endConditions, this.accelerationPropagator);

        // If we're already at the desired Mach speed, the acceleration segment is essentially empty, but we still want to add a state to the profile.
        // This is because the Mach speed target should be propagated from this state to all future states.
        if (step.length > 1) {
            step.last.speeds.speedTargetType = this.useMachTarget ? AccelFactorMode.CONSTANT_MACH : AccelFactorMode.CONSTANT_CAS;
            step.last.speeds.speedTarget = this.useMachTarget ? this.toMach : this.toSpeed;

            builder.push(step.last);
        } else {
            const copyOfLastState = copyState(builder.lastState);

            const shouldSpeedTargetTypeChange = this.useMachTarget && state.speeds.mach > this.endConditions.mach.max && state.speeds.speedTargetType === AccelFactorMode.CONSTANT_CAS;
            const shouldSpeedTargetChange = this.toMach > builder.lastState.speeds.speedTarget || this.toSpeed > builder.lastState.speeds.speedTarget;

            // Since we arrive at this segment because we reach the managed mach target, this acceleration segment will be basically empty.
            // However, we still need to place a state with Mach as speed target, since this will be propagated to the other
            if (shouldSpeedTargetTypeChange) {
                copyOfLastState.speeds.speedTargetType = AccelFactorMode.CONSTANT_MACH;
                copyOfLastState.speeds.speedTarget = this.toMach;
            } else if (shouldSpeedTargetChange) {
                copyOfLastState.speeds.speedTarget = this.toSpeed;
            } else {
                return;
            }

            builder.push(copyOfLastState);
        }
    }

    get repr() {
        return 'PureAccelerationSegment';
    }
}

function copyState(state: AircraftState): AircraftState {
    const copy = { ...state };
    copy.speeds = { ...state.speeds };
    copy.config = { ...state.config };

    return copy;
}
