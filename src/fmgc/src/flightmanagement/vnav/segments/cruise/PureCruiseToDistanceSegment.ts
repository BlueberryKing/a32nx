import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { constantPitchPropagator, FlightPathAnglePitchTarget, IntegrationEndConditions, Integrator } from '@fmgc/flightmanagement/vnav/integrators';

export class PureCruiseToDistanceSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

    propagator: (state: AircraftState) => AircraftState;

    constructor(context: SegmentContext, private toDistance: NauticalMiles, altitude: Feet) {
        super();

        this.endConditions = { distanceFromStart: { max: toDistance } };

        // 1. The reason we pass in an altiude instead of using the cruise altiude is
        // because it is possible that there is a cruise step before this segment which steps away from the programmed cruise altitude
        // 2. The >25000 part is specified by a reference. TODO: Add to ref book
        const useMachVsCas = altitude > 25000;

        this.propagator = constantPitchPropagator(new FlightPathAnglePitchTarget(0), context, 5, useMachVsCas);
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
