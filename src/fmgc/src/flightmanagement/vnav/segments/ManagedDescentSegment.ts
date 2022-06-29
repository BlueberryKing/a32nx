import { GeometricPathSegment } from '@fmgc/flightmanagement/vnav/segments/GeometricPathSegment';
import { IdlePathSegment } from '@fmgc/flightmanagement/vnav/segments/IdlePathSegment';
import { AltitudeConstraintType } from '@fmgc/guidance/lnav/legs';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { SegmentContext, ProfileBuilder, AircraftState } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { McduPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';
import { PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';
import { FmgcFlightPhase } from '@shared/flightphase';

export class ManagedDescentSegment extends ProfileSegment {
    /**
     * The reason this is an array is because we use one below the speed limit and one above the speed limit,
     * as well as one above the crossover altitude
     */
    private geometricSegments: GeometricPathSegment[] = [];

    private idleSegment: IdlePathSegment;

    constructor(private context: SegmentContext, private constraints: ConstraintReader) {
        super();

        const { managedDescentSpeed, managedDescentSpeedMach, cruiseAltitude, descentSpeedLimit } = context.observer.get();

        const crossoverAltitude = context.computeCrossoverAltitude(managedDescentSpeed, managedDescentSpeedMach);
        const options: Omit<PropagatorOptions, 'useMachVsCas'> = {
            stepSize: -5,
            windProfileType: WindProfileType.Descent,
        };

        this.geometricSegments = [
            new GeometricPathSegment(context, constraints, managedDescentSpeed, Math.min(crossoverAltitude, cruiseAltitude), { ...options, useMachVsCas: false }),
            new GeometricPathSegment(context, constraints, managedDescentSpeed, cruiseAltitude, { ...options, useMachVsCas: true }),
        ];

        if (Number.isFinite(descentSpeedLimit.underAltitude) && Number.isFinite(descentSpeedLimit.speed)) {
            this.geometricSegments.unshift(
                new GeometricPathSegment(
                    context,
                    constraints,
                    Math.min(descentSpeedLimit.speed,
                        managedDescentSpeed),
                    Math.min(descentSpeedLimit.underAltitude,
                        crossoverAltitude,
                        cruiseAltitude),
                    { ...options, useMachVsCas: false },
                ),
            );
        }

        this.idleSegment = new IdlePathSegment(context, constraints, cruiseAltitude);

        this.children = [
            ...this.geometricSegments,
            this.idleSegment,
        ];

        this.setGeometricPathPoint();
    }

    get repr(): string {
        return 'ManagedDescentSegment';
    }

    shouldCompute(_state: AircraftState, _builder: ProfileBuilder): boolean {
        return this.context.observer.get().flightPhase <= FmgcFlightPhase.Descent;
    }

    compute(_state: AircraftState, builder: ProfileBuilder): void {
        builder.changePhase(FmgcFlightPhase.Descent);
    }

    onAfterBuildingChildren(builder: ProfileBuilder): void {
        builder.requestPseudoWaypoint(McduPseudoWaypointType.TopOfDescent, builder.lastState);
    }

    /**
     * TODO: In reality, this should use an idle path to determine which constraint is constraining and use a geometric path to this.
     * For now, we just use the first at-or-below constraint.
    */
    private setGeometricPathPoint() {
        for (const constraint of this.constraints.descentAltitudeConstraints) {
            if (constraint.constraint.type !== AltitudeConstraintType.atOrAbove) {
                this.geometricSegments.forEach((segment) => segment.setLastConstraint(constraint));

                return;
            }
        }
    }
}
