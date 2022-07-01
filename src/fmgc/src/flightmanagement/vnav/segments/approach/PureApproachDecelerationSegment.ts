import {
    FlightPathAnglePitchTarget,
    IdleThrustSetting,
    IntegrationEndConditions,
    IntegrationPropagator,
    Integrator,
    PropagatorOptions,
    speedChangePropagator,
} from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureApproachDecelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private endConditions: IntegrationEndConditions;

    private propagator: IntegrationPropagator;

    constructor(private context: SegmentContext, private flightPathAngle: Degrees, private toSpeed: Knots, private toDistance: NauticalMiles, options: PropagatorOptions) {
        super();

        this.endConditions = {
            distanceFromStart: { min: toDistance },
            speed: { max: toSpeed },
        };

        this.propagator = speedChangePropagator(
            context,
            new IdleThrustSetting(this.context.atmosphericConditions),
            new FlightPathAnglePitchTarget(this.flightPathAngle),
            false,
            options,
        );
    }

    get repr(): string {
        return 'PureApproachDecelerationSegment';
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const step = this.integrator.integrate(
            state,
            this.endConditions,
            this.propagator,
        );

        if (step.length > 1) {
            builder.push(step.last);
        }
    }
}
