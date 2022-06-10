import { FlightPathAnglePitchTarget, IdleThrustSetting, IntegrationEndConditions, Integrator, speedChangePropagator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureInitialApproachDecelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private endConditions: IntegrationEndConditions;

    // This segment is only relevant if we are past a certain speed constraint already. It does nothing otherwise.
    constructor(private context: NodeContext, private flightPathAngle: Degrees, private toSpeed: Knots, private minDistance: NauticalMiles) {
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
            new IdleThrustSetting(this.context.atmosphericConditions),
            new FlightPathAnglePitchTarget(this.flightPathAngle),
            false,
            this.context,
            -5,
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
