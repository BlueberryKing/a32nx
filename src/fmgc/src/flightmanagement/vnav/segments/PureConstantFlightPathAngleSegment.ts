import { constantPitchPropagator, FlightPathAnglePitchTarget, IntegrationEndCondition, Integrator } from '@fmgc/flightmanagement/vnav/integrators';
import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class PureConstantFlightPathAngleSegment extends ProfileSegment {
    private integrator = new Integrator();

    private endConditions: IntegrationEndCondition[] = [];

    /**
     *
     * @param context
     * @param flightPathAngle
     * @param toDistance
     * @param maxAltiude
     */
    constructor(private context: NodeContext, private flightPathAngle: Degrees, private toDistance: NauticalMiles, private maxAltiude: Feet = context.observer.get().cruiseAltitude) {
        super();

        this.endConditions = [
            ({ distanceFromStart }) => distanceFromStart <= this.toDistance,
            ({ altitude }) => altitude >= this.maxAltiude,
        ];
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const pitchTarget = new FlightPathAnglePitchTarget(this.flightPathAngle);

        const descentPath = this.integrator.integrate(
            state,
            this.endConditions,
            constantPitchPropagator(pitchTarget, this.context, -0.1),
        );

        if (descentPath.length > 1) {
            builder.push(descentPath.last);
        }
    }

    get repr(): string {
        return `PureConstantFlightPathAngleSegment - Descend from ${this.toDistance.toFixed(2)} NM`;
    }
}
