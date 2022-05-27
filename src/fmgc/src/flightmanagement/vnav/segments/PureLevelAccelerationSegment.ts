import { FlightPathAnglePitchTarget, IntegrationEndCondition, IntegrationPropagator, Integrator, speedChangePropagator, ThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { NodeContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureLevelAccelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndCondition[] = [];

    private propagator: IntegrationPropagator

    constructor(
        private context: NodeContext, thrustSetting: ThrustSetting, private toSpeed: Knots, private toDistance: NauticalMiles,
    ) {
        super();

        this.endConditions = [
            (state) => state.speed > toSpeed,
            (state) => state.distanceFromStart > toDistance,
        ];

        const pitchTarget = new FlightPathAnglePitchTarget(0);

        this.propagator = speedChangePropagator(thrustSetting, pitchTarget, true, this.context);
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        if (state.distanceFromStart > this.toDistance) {
            return;
        }

        const endState = this.integrator.integrate(
            state,
            this.endConditions,
            this.propagator,
        ).last;

        builder.push(endState);
    }

    get repr() {
        return `PureLevelAccelerationSegment - Accelerate to Fly level to ${this.toDistance} NM `;
    }
}
