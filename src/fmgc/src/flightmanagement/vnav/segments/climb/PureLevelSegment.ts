import { constantPitchPropagator, FlightPathAnglePitchTarget, IntegrationEndConditions, Integrator, PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';
import { SegmentContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { NdPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';

export class PureLevelSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndConditions;

    constructor(
        private context: SegmentContext, private toDistance: NauticalMiles, private options: PropagatorOptions,
    ) {
        super();

        this.endConditions = { distanceFromStart: { max: toDistance } };
    }

    override compute(state: AircraftState, builder: ProfileBuilder) {
        const pitchTarget = new FlightPathAnglePitchTarget(0);

        const accelerationPath = this.integrator.integrate(
            state,
            this.endConditions,
            constantPitchPropagator(pitchTarget, this.context, this.options),
        );

        if (accelerationPath.length > 1) {
            builder.push(accelerationPath.last);
            builder.requestNdPseudoWaypoint(NdPseudoWaypointType.Level1Climb, accelerationPath.first);
            builder.requestNdPseudoWaypoint(NdPseudoWaypointType.StartOfClimb1, accelerationPath.last);
        }
    }

    get repr() {
        return `PureLevelSegmentSegment - Fly level to ${this.toDistance} NM`;
    }
}
