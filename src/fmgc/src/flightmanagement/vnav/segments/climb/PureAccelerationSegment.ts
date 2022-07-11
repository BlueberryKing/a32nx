import { accelerationPropagator, IntegrationEndConditions, Integrator, PropagatorOptions, ThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { SegmentContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { AccelFactorMode } from '@fmgc/guidance/vnav/common';

export class PureAccelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

    private useMachTarget: boolean = false;

    constructor(
        private context: SegmentContext,
        private thrustSetting: ThrustSetting,
        private toSpeed: Knots,
        private toAltitude: Feet,
        private options: PropagatorOptions,
        maxDistance: NauticalMiles = Infinity,
        toMach: Mach = 0.82,
    ) {
        super();

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
        context: SegmentContext,
        thrustSetting: ThrustSetting,
        toMach: Mach,
        toAltitude: Feet,
        options: PropagatorOptions,
        maxDistance: NauticalMiles = Infinity,
    ): PureAccelerationSegment {
        const segment = new PureAccelerationSegment(
            context,
            thrustSetting,
            340, // Just use VMO for this since we really want to constraint the Mach number
            toAltitude,
            options,
            maxDistance,
            toMach,
        );

        segment.useMachTarget = true;
        return segment;
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const step = this.integrator.integrate(state,
            this.endConditions,
            accelerationPropagator(this.thrustSetting, this.context, this.options));

        // We need to set a state here with Mach speedtarget, so the propagators building off of this state take the correct target
        // However, we only want to do this if we're above crossover altitude.
        if (step.length > 1) {
            step.last.speeds.speedTargetType = this.useMachTarget ? AccelFactorMode.CONSTANT_MACH : AccelFactorMode.CONSTANT_CAS;
            builder.push(step.last);
        } else if (state.speeds.mach > this.endConditions.mach.max) {
            const copyOfLastState = { ...builder.lastState };
            copyOfLastState.speeds.speedTargetType = this.useMachTarget ? AccelFactorMode.CONSTANT_MACH : AccelFactorMode.CONSTANT_CAS;

            builder.push(copyOfLastState);
        }
    }

    get repr() {
        return `PureAccelerationSegment - Accelerate to ${this.toSpeed} kts, stay below ${this.toAltitude} ft`;
    }
}
