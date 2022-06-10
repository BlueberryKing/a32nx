import { constantPitchPropagator, FlightPathAnglePitchTarget, IntegrationEndConditions, Integrator } from '@fmgc/flightmanagement/vnav/integrators';
import { NodeContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureLevelSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

    constructor(
        private context: NodeContext, private toDistance: NauticalMiles,
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
            constantPitchPropagator(pitchTarget, this.context),
        ).last;

        builder.push(endState);
    }

    get repr() {
        return `PureLevelSegmentNode - Fly level to ${this.toDistance} NM`;
    }
}
