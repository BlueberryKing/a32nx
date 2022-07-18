import { GeometricPathSegment } from '@fmgc/flightmanagement/vnav/segments/descent/GeometricPathSegment';
import { IdlePathSegment } from '@fmgc/flightmanagement/vnav/segments/descent/IdlePathSegment';
import { AltitudeConstraint, AltitudeConstraintType } from '@fmgc/guidance/lnav/legs';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { SegmentContext, ProfileBuilder, AircraftState, BuilderVisitor, TemporaryStateSequence } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { McduPseudoWaypointType, NdPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';
import { PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';
import { FmgcFlightPhase } from '@shared/flightphase';
import { MathUtils } from '@shared/MathUtils';

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

        const options: PropagatorOptions = {
            stepSize: -5,
            windProfileType: WindProfileType.Descent,
        };

        this.geometricSegments = [
            new GeometricPathSegment(context, constraints, managedDescentSpeed, Math.min(crossoverAltitude, cruiseAltitude), options),
            new GeometricPathSegment(context, constraints, managedDescentSpeed, cruiseAltitude, options),
        ];

        if (Number.isFinite(descentSpeedLimit.underAltitude) && Number.isFinite(descentSpeedLimit.speed) && descentSpeedLimit.speed < managedDescentSpeed) {
            this.geometricSegments.unshift(
                new GeometricPathSegment(
                    context,
                    constraints,
                    descentSpeedLimit.speed,
                    Math.min(descentSpeedLimit.underAltitude, crossoverAltitude, cruiseAltitude),
                    options,
                ),
            );
        }

        this.idleSegment = new IdlePathSegment(context, constraints, crossoverAltitude, cruiseAltitude);

        this.children = [
            ...this.geometricSegments,
            this.idleSegment,
        ];
    }

    get repr(): string {
        return 'ManagedDescentSegment';
    }

    shouldCompute(_state: AircraftState, _builder: ProfileBuilder): boolean {
        return this.context.observer.get().flightPhase <= FmgcFlightPhase.Descent;
    }

    compute(state: AircraftState, builder: ProfileBuilder): void {
        builder.changePhase(FmgcFlightPhase.Descent);

        const geometricPathPoint = this.findGeometricPathPoint(state);
        this.geometricSegments.forEach((segment) => segment.setGeometricPathPoint(geometricPathPoint));
    }

    onAfterBuildingChildren(builder: ProfileBuilder): void {
        builder.requestMcduPseudoWaypoint(McduPseudoWaypointType.TopOfDescent, builder.lastState);
        builder.requestNdPseudoWaypoint(NdPseudoWaypointType.TopOfDescent1, builder.lastState);
    }

    private findGeometricPathPoint(decelPoint: AircraftState): AircraftState {
        // Start at decel point
        let geometricPathPoint = decelPoint;

        // Set up idle path
        const idlePathBuilder = new ProfileBuilder(decelPoint, FmgcFlightPhase.Descent, true);
        const visitor = new BuilderVisitor(idlePathBuilder);

        // Build idle path from decel point
        this.idleSegment.accept(visitor);
        let idleProfile = new TemporaryStateSequence(...idlePathBuilder.allCheckpoints.slice().reverse());

        for (let i = this.constraints.descentAltitudeConstraints.length - 1; i >= 0; i--) {
            const constraint = this.constraints.descentAltitudeConstraints[i];
            if (constraint.distanceFromStart > geometricPathPoint.distanceFromStart) {
                continue;
            }

            const stateAtConstraint = idleProfile.interpolateEverythingFromStart(constraint.distanceFromStart);

            // If constraint is violated by idle path, move geometric path point to this constraint, and build new idle path from here
            const [isAltitudeConstraintMet, altitudeToContinueFrom] = this.evaluateAltitudeConstraint(stateAtConstraint.altitude, constraint.constraint);
            if (!isAltitudeConstraintMet) {
                geometricPathPoint = stateAtConstraint;
                geometricPathPoint.altitude = altitudeToContinueFrom;

                // Build new idle path
                idlePathBuilder.resetPhase().resetPseudoWaypoints().push(geometricPathPoint);
                this.idleSegment.accept(visitor);
                idleProfile = new TemporaryStateSequence(...idlePathBuilder.allCheckpoints.slice().reverse());
            }
        }

        return geometricPathPoint;
    }

    private evaluateAltitudeConstraint(altitude: Feet, constraint: AltitudeConstraint): [boolean, Feet] {
        switch (constraint.type) {
        case AltitudeConstraintType.at:
            return [Math.abs(altitude - constraint.altitude1) <= 250, MathUtils.clamp(altitude, constraint.altitude1 - 250, constraint.altitude1 + 250)];
        case AltitudeConstraintType.atOrAbove:
            return [(altitude - constraint.altitude1) >= -250, Math.max(altitude, constraint.altitude1 - 250)];
        case AltitudeConstraintType.atOrBelow:
            return [(altitude - constraint.altitude1) <= 250, Math.min(altitude, constraint.altitude1 + 250)];
        case AltitudeConstraintType.range:
            return [(altitude - constraint.altitude2) >= -250 && (altitude - constraint.altitude1) < 250, MathUtils.clamp(altitude, constraint.altitude2 - 250, constraint.altitude1 + 250)];
        default:
            console.error('[FMS/VNAV] Invalid altitude constraint type');
            return [true, altitude];
        }
    }
}
