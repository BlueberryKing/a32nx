import { FlightPathAnglePitchTarget, IdleThrustSetting, IntegrationPropagator, Integrator, speedChangePropagator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureGeometricDecelerationSegment extends ProfileSegment {
    private integrator = new Integrator();

    private propagator: IntegrationPropagator;

    constructor(context: NodeContext, private flightPathAngle: Degrees, private toSpeed: Knots, private toDistance: NauticalMiles) {
        super();

        this.propagator = speedChangePropagator(
            new IdleThrustSetting(context.atmosphericConditions),
            new FlightPathAnglePitchTarget(flightPathAngle),
            context,
            -0.1,
        );
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        // The AMM says decel distance is max 20 NM and goes max 6000 higher.
        const endConditions = [
            ({ distanceFromStart }) => distanceFromStart <= Math.max(this.toDistance, state.distanceFromStart - 20),
            ({ altitude }) => altitude >= state.altitude + 6000,
            ({ speed }) => speed >= this.toSpeed,
        ];

        const decelerationPath = this.integrator.integrate(
            state,
            endConditions,
            this.propagator,
        );

        if (decelerationPath.length > 1) {
            builder.push(decelerationPath.last);
        }
    }

    get repr(): string {
        return `PureGeometricDecelerationSegment - Decelerate to ${this.toSpeed} kts`;
    }
}