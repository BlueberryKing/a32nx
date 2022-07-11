import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { constantPitchPropagator, FlightPathAnglePitchTarget, IntegrationEndConditions, Integrator, PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';

export class PureCruiseToDistanceSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

    propagator: (state: AircraftState) => AircraftState;

    constructor(context: SegmentContext, toDistance: NauticalMiles, options: PropagatorOptions) {
        super();

        this.endConditions = { distanceFromStart: { max: toDistance } };

        this.propagator = constantPitchPropagator(new FlightPathAnglePitchTarget(0), context, options);
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const segment = this.integrator.integrate(
            state,
            this.endConditions,
            this.propagator,
        );

        if (segment.length > 1) {
            builder.push(segment.last);
        }
    }

    get repr(): string {
        return 'PureCruiseToDistanceSegment';
    }
}
