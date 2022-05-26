import { AircraftState, NodeContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { Step } from '@fmgc/guidance/vnav/StepCoordinator';
import { ClimbThrustSetting, constantPitchPropagator, constantThrustPropagator, IntegrationEndCondition, Integrator, VerticalSpeedPitchTarget } from '@fmgc/flightmanagement/vnav/integrators';
import { McduPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';

/**
 * A step climb or descent
 */

export class PureCruiseStepSegment extends ProfileSegment {
    private integrator: Integrator = new Integrator();

    private readonly endConditions: IntegrationEndCondition[] = [];

    propagator: (state: AircraftState) => AircraftState;

    private isClimbVsDescent: boolean;

    constructor(context: NodeContext, private step: Step, private fromAltitude: Feet, private maxDistance: NauticalMiles) {
        super();

        this.isClimbVsDescent = step.toAltitude > fromAltitude;

        // TODO: Figure out whether to use Mach or IAS target
        this.propagator = this.isClimbVsDescent
            ? constantThrustPropagator(new ClimbThrustSetting(context.atmosphericConditions), context)
            : constantPitchPropagator(new VerticalSpeedPitchTarget(-1000), context);

        this.endConditions = [
            ({ distanceFromStart }) => distanceFromStart >= maxDistance,
            ({ altitude }) => (this.isClimbVsDescent ? altitude >= step.toAltitude : altitude <= step.toAltitude),
        ];
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        builder.requestPseudoWaypoint(
            this.isClimbVsDescent ? McduPseudoWaypointType.StepClimb : McduPseudoWaypointType.StepDescent,
            state,
        );

        const step = this.integrator.integrate(
            state,
            this.endConditions,
            this.propagator,
        );

        if (step.length > 1) {
            builder.push(step.last);

            // I think there's only an indication on the MCDU for a step climb, not a step descent.
            if (this.isClimbVsDescent) {
                builder.requestPseudoWaypoint(McduPseudoWaypointType.TopOfClimb, state);
            }
        }
    }

    get repr(): string {
        return `PureCruiseStepSegment - Step from ${Math.round(this.fromAltitude).toFixed(0)} ft to ${Math.round(this.step.toAltitude).toFixed(0)} ft.`;
    }
}
