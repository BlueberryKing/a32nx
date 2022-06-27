import { constantPitchPropagator, FlightPathAnglePitchTarget, IntegrationEndConditions, IntegrationPropagator, Integrator, PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

/**
 * Represents Segment from Final Descent Point to 1000 ft above the threshold
 */

export class FinalApproachSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private endConditions: IntegrationEndConditions;

    private propagator: IntegrationPropagator;

    constructor(context: SegmentContext, options: PropagatorOptions) {
        super();

        const { destinationAirfieldElevation } = context.observer.get();
        this.endConditions = { altitude: { max: destinationAirfieldElevation + 1000 } };

        this.propagator = constantPitchPropagator(
            new FlightPathAnglePitchTarget(-3),
            context,
            options,
        );
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
        return 'FinalApproachSegment';
    }
}
