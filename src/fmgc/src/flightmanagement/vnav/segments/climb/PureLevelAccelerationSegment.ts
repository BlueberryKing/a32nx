import {
    FlightPathAnglePitchTarget,
    IntegrationEndConditions,
    IntegrationPropagator,
    Integrator,
    PropagatorOptions,
    speedChangePropagator,
    ThrustSetting,
} from '@fmgc/flightmanagement/vnav/integrators';
import { SegmentContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureLevelAccelerationSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

    private propagator: IntegrationPropagator

    constructor(
        private context: SegmentContext, thrustSetting: ThrustSetting, private toSpeed: Knots, private toDistance: NauticalMiles, options: PropagatorOptions,
    ) {
        super();

        this.endConditions = {
            calibratedAirspeed: { max: toSpeed },
            distanceFromStart: { max: toDistance },
        };

        const pitchTarget = new FlightPathAnglePitchTarget(0);

        this.propagator = speedChangePropagator(this.context, thrustSetting, pitchTarget, true, options);
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        if (state.distanceFromStart > this.toDistance) {
            return;
        }

        const endState = this.integrator.integrate(
            state,
            this.endConditions,
            this.propagator,
        ).last;

        builder.push(endState);
    }

    get repr() {
        return `PureLevelAccelerationSegment - Accelerate to Fly level to ${this.toDistance} NM `;
    }
}
