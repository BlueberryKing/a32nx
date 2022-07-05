import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { MathUtils } from '@shared/MathUtils';
import { DescentAltitudeConstraintSegment } from '@fmgc/flightmanagement/vnav/segments/descent/DescentAltitudeConstraintSegment';
import { AircraftState, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { AltitudeConstraintType } from '@fmgc/guidance/lnav/legs';
import { PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';

export class GeometricPathSegment extends ProfileSegment {
    /**
     * The `lastConstraint` is the first constraint encountered along the lateral track.
     */
    private geometricPathPoint?: AircraftState = null;

    /**
     *
     * @param context
     * @param constraints
     * @param maxSpeed
     * @param toAltitude This is used to implement the speed limit.
     */
    constructor(private context: SegmentContext, private constraints: ConstraintReader, private maxSpeed: Knots, private toAltitude: Feet, private options: PropagatorOptions) {
        super();
    }

    /**
     * The geometric path point is the point separating the idle path from the geometric path
     * @param geometricPathPoint
     */
    setGeometricPathPoint(geometricPathPoint: AircraftState) {
        this.geometricPathPoint = geometricPathPoint;
    }

    compute(state: AircraftState, _builder: ProfileBuilder): void {
        this.children = [];

        if (this.geometricPathPoint === null) {
            return;
        }

        const plannedSegments: PlannedGeometricSegment[] = [];
        this.planDescentSegments(state, this.geometricPathPoint, plannedSegments);

        for (const plannedSegment of plannedSegments) {
            const fpa = MathUtils.RADIANS_TO_DEGREES * Math.atan(plannedSegment.gradient / 6076.12);

            this.children.push(
                new DescentAltitudeConstraintSegment(
                    this.context, this.constraints, plannedSegment.end.distanceFromStart, fpa, this.maxSpeed, this.toAltitude, this.options,
                ),
            );
        }
    }

    private planDescentSegments(start: GeometricPathPoint, end: GeometricPathPoint, segments: PlannedGeometricSegment[]): PlannedGeometricSegment[] {
        // A "gradient" is just a quantity of units Feet / NauticalMiles
        const gradient = this.calculateGradient(start, end);
        const constraints = this.constraints.descentAltitudeConstraints;

        for (let i = 0; i < constraints.length; i++) {
            const constraint = constraints[i];

            if (constraint.distanceFromStart >= start.distanceFromStart || constraint.distanceFromStart < end.distanceFromStart) {
                continue;
            }

            const altAtConstraint = start.altitude + gradient * (constraint.distanceFromStart - start.distanceFromStart);
            const constraintType = constraint.constraint.type;

            if (constraintType === AltitudeConstraintType.at && Math.abs(altAtConstraint - constraint.constraint.altitude1) > 250) {
                const center = { distanceFromStart: constraint.distanceFromStart, altitude: constraint.constraint.altitude1 };

                this.planDescentSegments(start, center, segments);
                this.planDescentSegments(center, end, segments);

                return;
            } if (constraintType === AltitudeConstraintType.atOrAbove && constraint.constraint.altitude1 - altAtConstraint > 250) {
                const center = { distanceFromStart: constraint.distanceFromStart, altitude: constraint.constraint.altitude1 };

                this.planDescentSegments(start, center, segments);
                this.planDescentSegments(center, end, segments);

                return;
            } if (constraintType === AltitudeConstraintType.atOrBelow && altAtConstraint - constraint.constraint.altitude1 > 250) {
                const center = { distanceFromStart: constraint.distanceFromStart, altitude: constraint.constraint.altitude1 };

                this.planDescentSegments(start, center, segments);
                this.planDescentSegments(center, end, segments);

                return;
            } if (constraintType === AltitudeConstraintType.range && (constraint.constraint.altitude2 - altAtConstraint > 250 || altAtConstraint - constraint.constraint.altitude1 > 250)) {
                const altitude = Math.min(constraint.constraint.altitude2, Math.max(altAtConstraint, constraint.constraint.altitude1));
                const center = { distanceFromStart: constraint.distanceFromStart, altitude };

                this.planDescentSegments(start, center, segments);
                this.planDescentSegments(center, end, segments);

                return;
            }
        }

        segments.push({ end, gradient });
    }

    calculateGradient(start: GeometricPathPoint, end: GeometricPathPoint) {
        return Math.abs(start.distanceFromStart - end.distanceFromStart) < 1e-12
            ? 0
            : (start.altitude - end.altitude) / (start.distanceFromStart - end.distanceFromStart);
    }

    get repr(): string {
        return 'GeometricPathSegment';
    }
}

export type GeometricPathPoint = {
    distanceFromStart: NauticalMiles,
    altitude: Feet
}

type PlannedGeometricSegment = {
    gradient: number,
    end: GeometricPathPoint,
}
