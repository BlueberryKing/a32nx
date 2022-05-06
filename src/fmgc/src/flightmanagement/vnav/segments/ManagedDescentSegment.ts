import { constantThrustPropagator, IdleThrustSetting, IntegrationEndCondition, IntegrationPropagator, Integrator } from '@fmgc/flightmanagement/vnav/integrators';
import { GeometricPathSegment } from '@fmgc/flightmanagement/vnav/segments/GeometricPathSegment';
import { IdlePathSegment } from '@fmgc/flightmanagement/vnav/segments/IdlePathSegment';
import { AltitudeConstraintType } from '@fmgc/guidance/lnav/legs';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { NodeContext, AircraftState, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class ManagedDescentSegment extends ProfileSegment {
    private integrator = new Integrator();

    private geometricSegment: GeometricPathSegment;

    private idleSegment: IdlePathSegment;

    private readonly endConditions: IntegrationEndCondition[] = [];

    private idleThrustPropagator: IntegrationPropagator;

    constructor(context: NodeContext, private constraints: ConstraintReader) {
        super();

        const { managedDescentSpeed, cruiseAltitude } = context.observer.get();

        this.geometricSegment = new GeometricPathSegment(context, constraints, managedDescentSpeed);
        this.idleSegment = new IdlePathSegment(context, constraints, cruiseAltitude);

        this.children = [
            this.geometricSegment,
            this.idleSegment,
        ];

        this.endConditions = [
            ({ altitude }) => altitude > cruiseAltitude,
        ];

        this.idleThrustPropagator = constantThrustPropagator(
            new IdleThrustSetting(context.atmosphericConditions),
            context,
            -1,
        );
    }

    get repr(): string {
        return 'DescentSegment';
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        // TODO: Actually use an idle path to check which constraint cannot be met using an idle path.
        if (this.constraints.descentAltitudeConstraints.length > 0 && this.constraints.descentAltitudeConstraints[0].constraint.type !== AltitudeConstraintType.atOrAbove) {
            this.geometricSegment.setLastConstraint(this.constraints.descentAltitudeConstraints[0]);
        }

        return;

        // Try idle
        const result = this.integrator.integrate(
            state,
            this.endConditions,
            this.idleThrustPropagator,
        );

        // Check which alt constraint violates
        for (const constraint of this.constraints.descentAltitudeConstraints) {
            // If constraint does not lie on idle path at all, we don't care
            if (constraint.distanceFromStart > state.distanceFromStart || constraint.distanceFromStart < result.last.distanceFromStart) {
                continue;
            }

            const altAtConstraint = result.reverse().interpolateEverythingFromStart(constraint.distanceFromStart);
            if (constraint.constraint.type !== AltitudeConstraintType.atOrAbove && constraint.constraint.altitude1 < altAtConstraint) {
                this.geometricSegment.setLastConstraint(constraint);
            }
        }
    }
}
