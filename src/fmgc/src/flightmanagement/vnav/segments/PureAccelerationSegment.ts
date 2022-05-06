import { accelerationPropagator, IntegrationEndCondition, Integrator, ThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { NodeContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureAccelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndCondition[] = [];

    constructor(
        private context: NodeContext, private thrustSetting: ThrustSetting, private toSpeed: Knots, private toAltitude: Feet, maxDistance: NauticalMiles = Infinity,
    ) {
        super();

        this.endConditions = [
            (state) => state.speed > toSpeed,
            (state) => state.altitude > toAltitude,
            (state) => state.distanceFromStart > maxDistance,
        ];
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const endState = this.integrator.integrate(state,
            this.endConditions,
            accelerationPropagator(this.thrustSetting, this.context)).last;

        // Only add the new state if there was a significant change
        if (endState.distanceFromStart - state.distanceFromStart > 1e-4) {
            builder.push(endState);
        }
    }

    get repr() {
        return `PureAccelerationNode - Accelerate to ${this.toSpeed} kts, stay below ${this.toAltitude} ft`;
    }
}
