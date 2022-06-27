import {
    IdleThrustSetting,
    IntegrationEndConditions,
    IntegrationPropagator,
    Integrator,
    speedChangePropagator,
    VerticalSpeedPitchTarget,
} from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureIdlePathDecelerationSegment extends ProfileSegment {
    private readonly endConditions: IntegrationEndConditions;

    private integrator = new Integrator();

    private idleThrustPropagator: IntegrationPropagator;

    constructor(context: SegmentContext, private toAltitude: Feet, private toSpeed: Knots, private toDistance: NauticalMiles) {
        super();

        this.endConditions = {
            altitude: { max: toAltitude },
            speed: { max: toSpeed },
            distanceFromStart: { min: toDistance },
        };

        // TODO: The pitch target should not be -500 fpm
        this.idleThrustPropagator = speedChangePropagator(
            new IdleThrustSetting(context.atmosphericConditions),
            new VerticalSpeedPitchTarget(-500),
            false,
            context,
            -5,
        );
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const decelerationStep = this.integrator.integrate(
            state,
            this.endConditions,
            this.idleThrustPropagator,
        );

        if (decelerationStep.length > 1) {
            builder.push(decelerationStep.last);
        }
    }

    get repr(): string {
        return 'PureIdlePathDecelerationSegment';
    }
}
