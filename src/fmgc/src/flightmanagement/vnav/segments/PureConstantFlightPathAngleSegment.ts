import { constantPitchPropagator, FlightPathAnglePitchTarget, Integrator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureConstantFlightPathAngleSegment extends ProfileSegment {
    private integrator = new Integrator();

    constructor(private context: NodeContext, private flightPathAngle: Degrees, private toDistance: NauticalMiles) {
        super();
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const endConditions = [
            ({ distanceFromStart }) => distanceFromStart > this.toDistance,
        ];

        const pitchTarget = new FlightPathAnglePitchTarget(this.flightPathAngle);

        const descentPath = this.integrator.integrate(
            state,
            endConditions,
            constantPitchPropagator(pitchTarget, this.context),
        );

        builder.push(descentPath.last);
    }
}
