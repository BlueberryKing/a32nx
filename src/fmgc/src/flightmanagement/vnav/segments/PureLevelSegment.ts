import { constantPitchPropagator, FlightPathAnglePitchTarget, IntegrationEndCondition, Integrator } from '@fmgc/flightmanagement/vnav/integrators';
import { ProfileSegment, NodeContext, AircraftState, ProfileBuilder } from './index';

export class PureLevelSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndCondition[] = [];

    constructor(
        private context: NodeContext, private toDistance: NauticalMiles,
    ) {
        super();

        this.endConditions = [
            (state) => state.distanceFromStart > toDistance,
        ];
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
        );

        builder.push(endState);
    }

    get repr() {
        return `PureLevelSegmentNode - Fly level to ${this.toDistance} NM`;
    }
}
