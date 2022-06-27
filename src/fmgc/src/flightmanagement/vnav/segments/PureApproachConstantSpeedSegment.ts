import { constantPitchPropagator, FlightPathAnglePitchTarget, IntegrationEndConditions, Integrator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureApproachConstantSpeedSegment extends ProfileSegment {
    private integrator = new Integrator();

    constructor(private context: SegmentContext, private flightPathAngle: Degrees, private toDistance: NauticalMiles, private toSpeed: Knots) {
        super();
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const endConditions: IntegrationEndConditions = {
            distanceFromStart: { min: this.toDistance },
            speed: { max: this.toSpeed },
        };

        const pitchTarget = new FlightPathAnglePitchTarget(this.flightPathAngle);

        const descentPath = this.integrator.integrate(
            state,
            endConditions,
            constantPitchPropagator(pitchTarget, this.context, -5),
        );

        if (descentPath.length > 1) {
            builder.push(descentPath.last);
        }
    }

    get repr(): string {
        return 'PureApproachConstantSpeedSegment';
    }
}
