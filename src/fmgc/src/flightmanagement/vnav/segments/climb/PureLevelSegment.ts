import { constantPitchPropagator, FlightPathAnglePitchTarget, IntegrationEndConditions, Integrator, PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';
import { SegmentContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureLevelSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

    constructor(
        private context: SegmentContext, private toDistance: NauticalMiles, private options: PropagatorOptions,
    ) {
        super();

        this.endConditions = { distanceFromStart: { max: toDistance } };
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const pitchTarget = new FlightPathAnglePitchTarget(0);

        if (state.distanceFromStart > this.toDistance) {
            return;
        }

        const endState = this.integrator.integrate(
            state,
            this.endConditions,
            constantPitchPropagator(pitchTarget, this.context, this.options),
        ).last;

        builder.push(endState);
    }

    get repr() {
        return `PureLevelSegmentSegment - Fly level to ${this.toDistance} NM`;
    }
}
