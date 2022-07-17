import { IntegrationEndConditions, IntegrationPropagator, Integrator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureClimbToAltitudeSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

    constructor(
        private climbPropagator: IntegrationPropagator,
        private toAltitude: Feet,
        maxDistance: NauticalMiles = Infinity,
    ) {
        super();

        this.endConditions = {
            altitude: { max: toAltitude },
            distanceFromStart: { max: maxDistance },
        };
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const step = this.integrator.integrate(
            state,
            this.endConditions,
            this.climbPropagator,
        );

        // Only add the new state if there was a significant change
        if (step.length > 1) {
            builder.push(...step.allButFirst());
        }
    }

    get repr() {
        return `PureClimbToAltitudeSegment - Climb to ${this.toAltitude} ft`;
    }
}
