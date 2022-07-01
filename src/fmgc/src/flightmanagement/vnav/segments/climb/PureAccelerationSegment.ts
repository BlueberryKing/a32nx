import { accelerationPropagator, IntegrationEndConditions, Integrator, PropagatorOptions, ThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { SegmentContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureAccelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

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
            speed: { max: toSpeed },
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
        return new PureAccelerationSegment(
            context,
            thrustSetting,
            340, // Just use VMO for this since we really want to constraint the Mach number
            toAltitude,
            options,
            maxDistance,
            toMach,
        );
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const step = this.integrator.integrate(state,
            this.endConditions,
            accelerationPropagator(this.thrustSetting, this.context, this.options));

        // Only add the new state if there was a significant change
        if (step.length > 1) {
            builder.push(step.last);
        }
    }

    get repr() {
        return `PureAccelerationSegment - Accelerate to ${this.toSpeed} kts, stay below ${this.toAltitude} ft`;
    }
}
