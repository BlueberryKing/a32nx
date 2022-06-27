import { FlightPathAnglePitchTarget, IdleThrustSetting, IntegrationEndConditions, Integrator, PropagatorOptions, speedChangePropagator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureInitialApproachDecelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private endConditions: IntegrationEndConditions;

    // This segment is only relevant if we are past a certain speed constraint already. It does nothing otherwise.
    constructor(private context: SegmentContext, private flightPathAngle: Degrees, private toSpeed: Knots, private minDistance: NauticalMiles, private options: PropagatorOptions) {
        super();

        this.endConditions = { speed: { max: this.toSpeed } };
    }

    get repr(): string {
        return 'PureInitialApproachDecelerationSegment';
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        if (state.distanceFromStart > this.minDistance) {
            return;
        }

        const propagator = speedChangePropagator(
            this.context,
            new IdleThrustSetting(this.context.atmosphericConditions),
            new FlightPathAnglePitchTarget(this.flightPathAngle),
            false,
            this.options,
        );

        const step = this.integrator.integrate(
            state,
            this.endConditions,
            propagator,
        );

        if (step.length > 1) {
            builder.push(step.last);
        }
    }
}
