import { constantThrustPropagator, IntegrationEndConditions, Integrator, ThrustSetting } from '@fmgc/flightmanagement/vnav/integrators';
import { NodeContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureClimbToAltitudeSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

    constructor(
        private context: NodeContext, private thrustSetting: ThrustSetting, private toAltitude: Feet, private useMachVsCas: boolean = false, maxDistance: NauticalMiles = Infinity,
    ) {
        super();

        this.endConditions = {
            altitude: { max: toAltitude },
            distanceFromStart: { max: maxDistance },
        };
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const step = this.integrator.integrate(state,
            this.endConditions,
            constantThrustPropagator(this.thrustSetting, this.context, 5, this.useMachVsCas));

        // Only add the new state if there was a significant change
        if (step.length > 1) {
            builder.push(step.last);
        }
    }

    get repr() {
        return `PureClimbToAltitudeNode - Climb to ${this.toAltitude} ft`;
    }
}
