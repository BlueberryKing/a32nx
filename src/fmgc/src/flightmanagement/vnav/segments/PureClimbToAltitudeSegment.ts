import { constantThrustPropagator, IntegrationEndCondition, Integrator, ThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { ProfileSegment, NodeContext, AircraftState, ProfileBuilder } from './index';

export class PureClimbToAltitudeSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndCondition[] = [];

    constructor(
        private context: NodeContext, private thrustSetting: ThrustSetting, private toAltitude: Feet, maxDistance: NauticalMiles = Infinity,
    ) {
        super();

        this.endConditions = [
            (state) => state.altitude > toAltitude,
            (state) => state.distanceFromStart > maxDistance,
        ];
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const endState = this.integrator.integrate(state,
            this.endConditions,
            constantThrustPropagator(this.thrustSetting, this.context));

        // Only add the new state if there was a significant change
        if (endState.distanceFromStart - state.distanceFromStart > 1e-4) {
            builder.push(endState);
        }
    }

    get repr() {
        return `PureClimbToAltitudeNode - Climb to ${this.toAltitude} ft`;
    }
}
