import {
    constantThrustPropagator,
    IdleThrustSetting,
    IntegrationEndCondition,
    IntegrationPropagator,
    Integrator,
} from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureIdlePathConstantSpeedSegment extends ProfileSegment {
    private readonly endConditions: IntegrationEndCondition[] = [];

    private integrator = new Integrator();

    private idleThrustPropagator: IntegrationPropagator;

    constructor(context: NodeContext, private toAltitude: Feet, private toSpeed: Knots, private toDistance: NauticalMiles) {
        super();

        this.endConditions = [
            ({ altitude }) => altitude >= toAltitude,
            ({ distanceFromStart }) => distanceFromStart <= toDistance,
        ];

        this.idleThrustPropagator = constantThrustPropagator(
            new IdleThrustSetting(context.atmosphericConditions),
            context,
            -1,
        );
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const step = this.integrator.integrate(
            state,
            this.endConditions,
            this.idleThrustPropagator,
        );

        if (step.length > 1) {
            builder.push(step.last);
        }
    }

    get repr(): string {
        return 'PureIdlePathConstantSpeedSegment';
    }
}
