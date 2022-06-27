import {
    constantThrustPropagator,
    IdleThrustSetting,
    IntegrationEndConditions,
    IntegrationPropagator,
    Integrator,
} from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';

export class PureIdlePathConstantMachSegment extends ProfileSegment {
    private readonly endConditions: IntegrationEndConditions;

    private integrator = new Integrator();

    private idleThrustPropagator: IntegrationPropagator;

    constructor(context: SegmentContext, private toAltitude: Feet, private toDistance: NauticalMiles) {
        super();

        this.endConditions = {
            altitude: { max: toAltitude },
            distanceFromStart: { min: toDistance },
        };

        this.idleThrustPropagator = constantThrustPropagator(
            new IdleThrustSetting(context.atmosphericConditions),
            context,
            {
                stepSize: -5,
                windProfileType: WindProfileType.Descent,
                useMachVsCas: true,
            },
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
        return 'PureIdlePathConstantMachSegment';
    }
}
