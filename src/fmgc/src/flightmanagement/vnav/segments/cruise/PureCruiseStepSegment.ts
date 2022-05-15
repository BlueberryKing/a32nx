import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { Step } from '@fmgc/guidance/vnav/StepCoordinator';
import { ClimbThrustSetting, constantPitchPropagator, constantThrustPropagator, IntegrationEndCondition, Integrator, VerticalSpeedPitchTarget } from '@fmgc/flightmanagement/vnav/integrators';

/**
 * A step climb or descent
 */

export class PureCruiseStepSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndCondition[] = [];

    propagator: (state: AircraftState) => AircraftState;

    constructor(context: NodeContext, private step: Step, private fromAltitude: Feet, private maxDistance: NauticalMiles) {
        super();

        const isClimbVsDescent = step.toAltitude > fromAltitude;

        // TODO: Figure out whether to use Mach or IAS target
        this.propagator = isClimbVsDescent
            ? constantThrustPropagator(new ClimbThrustSetting(context.atmosphericConditions), context)
            : constantPitchPropagator(new VerticalSpeedPitchTarget(-1000), context);

        this.endConditions = [
            ({ distanceFromStart }) => distanceFromStart >= maxDistance,
            ({ altitude }) => isClimbVsDescent ? altitude >= step.toAltitude : altitude <= step.toAltitude
        ];
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        const step = this.integrator.integrate(
            state,
            this.endConditions,
            this.propagator
        );

        if (step.length > 1) {
            builder.push(step.last);
        }
    }

    get repr(): string {
        return `PureCruiseStepSegment - Step from ${Math.round(this.fromAltitude).toFixed(0)} ft to ${Math.round(this.step.toAltitude).toFixed(0)} ft.`;
    }
}
