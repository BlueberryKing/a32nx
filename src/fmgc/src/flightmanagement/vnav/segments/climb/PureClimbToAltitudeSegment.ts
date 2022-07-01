import { constantThrustPropagator, IntegrationEndConditions, Integrator, PropagatorOptions, ThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { SegmentContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureClimbToAltitudeSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

    constructor(
        private context: SegmentContext,
        private thrustSetting: ThrustSetting,
        private toAltitude: Feet,
        private options: PropagatorOptions,
        maxDistance: NauticalMiles = Infinity,
    ) {
        super();

        this.endConditions = {
            altitude: { max: toAltitude },
            distanceFromStart: { max: maxDistance },
        };
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const step = this.integrator.integrate(state,
            this.endConditions,
            constantThrustPropagator(this.thrustSetting, this.context, this.options));

        // Only add the new state if there was a significant change
        if (step.length > 1) {
            builder.push(...step.allButFirst());
        }
    }

    get repr() {
        return `PureClimbToAltitudeNode - Climb to ${this.toAltitude} ft`;
    }
}
