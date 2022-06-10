import { FlightPathAnglePitchTarget, IdleThrustSetting, IntegrationEndConditions, Integrator, speedChangePropagator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureApproachDecelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private endConditions: IntegrationEndConditions;

    constructor(private context: NodeContext, private flightPathAngle: Degrees, private toSpeed: Knots, private toDistance: NauticalMiles) {
        super();

        this.endConditions = {
            distanceFromStart: { min: toDistance },
            speed: { max: toSpeed },
        };
    }

    get repr(): string {
        return 'PureApproachDecelerationSegment';
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
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
