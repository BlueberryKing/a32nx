import {
    constantThrustPropagator,
    IdleThrustSetting,
    IntegrationEndConditions,
    IntegrationPropagator,
    Integrator,
    PropagatorOptions,
} from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureIdlePathConstantSpeedSegment extends ProfileSegment {
    private readonly endConditions: IntegrationEndConditions;

    private integrator = new Integrator();

    private idleThrustPropagator: IntegrationPropagator;

    constructor(context: SegmentContext, private toAltitude: Feet, private toDistance: NauticalMiles, options: PropagatorOptions) {
        super();

        this.endConditions = {
            altitude: { max: toAltitude },
            distanceFromStart: { min: toDistance },
        };

        this.idleThrustPropagator = constantThrustPropagator(
            new IdleThrustSetting(context.atmosphericConditions),
            context,
            options,
        );
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const step = this.integrator.integrate(
            state,
            this.endConditions,
            this.idleThrustPropagator,
        );

        if (step.length > 1) {
            builder.push(...step.allButFirst());
        }
    }

    get repr(): string {
        return 'PureIdlePathConstantSpeedSegment';
    }
}
