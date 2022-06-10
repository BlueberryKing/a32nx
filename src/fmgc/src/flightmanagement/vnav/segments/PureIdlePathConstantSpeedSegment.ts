import {
    constantThrustPropagator,
    IdleThrustSetting,
    IntegrationEndConditions,
    IntegrationPropagator,
    Integrator,
} from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureIdlePathConstantSpeedSegment extends ProfileSegment {
    private readonly endConditions: IntegrationEndConditions;

    private integrator = new Integrator();

    private idleThrustPropagator: IntegrationPropagator;

    constructor(context: NodeContext, private toAltitude: Feet, private toDistance: NauticalMiles) {
        super();

        this.endConditions = {
            altitude: { max: toAltitude },
            distanceFromStart: { min: toDistance },
        };

        this.idleThrustPropagator = constantThrustPropagator(
            new IdleThrustSetting(context.atmosphericConditions),
            context,
            -5,
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
