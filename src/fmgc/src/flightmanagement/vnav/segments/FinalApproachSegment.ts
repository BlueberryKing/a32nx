import { constantPitchPropagator, FlightPathAnglePitchTarget, IntegrationEndCondition, IntegrationPropagator, Integrator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

/**
 * Represents Segment from Final Descent Point to 1000 ft above the threshold
 */

export class FinalApproachSegment extends ProfileSegment {
    private integrator: Integrator;

    private endConditions: IntegrationEndCondition[];

    private propagator: IntegrationPropagator;

    constructor(context: NodeContext) {
        super();

        const { destinationAirfieldElevation } = context.observer.get();

        this.integrator = new Integrator();

        this.endConditions = [
            ({ altitude }) => altitude > destinationAirfieldElevation + 1000,
        ];

        this.propagator = constantPitchPropagator(
            new FlightPathAnglePitchTarget(-3),
            context,
            -0.1,
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
