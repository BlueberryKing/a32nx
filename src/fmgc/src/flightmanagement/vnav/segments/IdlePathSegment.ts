import { constantThrustPropagator, IdleThrustSetting, IntegrationEndCondition, IntegrationPropagator, Integrator } from '@fmgc/flightmanagement/vnav/integrators';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class IdlePathSegment extends ProfileSegment {
    private readonly endConditions: IntegrationEndCondition[] = [];

    private integrator = new Integrator();

    private idleThrustPropagator: IntegrationPropagator;

    constructor(context: NodeContext, private constraints: ConstraintReader, toAltitude: Feet, maxSpeed: Knots) {
        super();

        this.endConditions = [
            ({ altitude }) => altitude > toAltitude,
        ];

        this.idleThrustPropagator = constantThrustPropagator(
            new IdleThrustSetting(context.atmosphericConditions),
            context,
            -1,
        );
    }

    get repr(): string {
        return 'IdlePathSegment';
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        // Try idle
        const result = this.integrator.integrate(
            state,
            this.endConditions,
            this.idleThrustPropagator,
        );

        builder.push(result.last);
    }
}
