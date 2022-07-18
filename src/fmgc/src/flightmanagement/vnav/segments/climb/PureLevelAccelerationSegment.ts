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
import { NdPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';

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

        const accelerationPath = this.integrator.integrate(
            state,
            this.endConditions,
            this.propagator,
        );

        if (accelerationPath.length > 1) {
            builder.push(accelerationPath.last);
            builder.requestNdPseudoWaypoint(NdPseudoWaypointType.Level1Climb, accelerationPath.first);
            builder.requestNdPseudoWaypoint(NdPseudoWaypointType.SpeedChange1, accelerationPath.first);
            builder.requestNdPseudoWaypoint(NdPseudoWaypointType.StartOfClimb1, accelerationPath.last);
        }
    }

    get repr() {
        return `PureLevelAccelerationSegment - Accelerate to Fly level to ${this.toDistance} NM `;
    }
}
