import { FlightPathAnglePitchTarget, IdleThrustSetting, IntegrationEndCondition, Integrator, speedChangePropagator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureApproachDecelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private endConditions: IntegrationEndCondition[];

    constructor(private context: NodeContext, private flightPathAngle: Degrees, private toSpeed: Knots, private toDistance) {
        super();

        this.endConditions = [
            ({ distanceFromStart }) => distanceFromStart <= toDistance,
            ({ speed }) => speed >= toSpeed,
        ];
    }

    get repr(): string {
        return 'PureApproachDecelerationSegment';
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const propagator = speedChangePropagator(
            new IdleThrustSetting(this.context.atmosphericConditions),
            new FlightPathAnglePitchTarget(this.flightPathAngle),
            this.context,
            -0.1,
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
