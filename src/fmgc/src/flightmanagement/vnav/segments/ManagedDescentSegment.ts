import { GeometricPathSegment } from '@fmgc/flightmanagement/vnav/segments/GeometricPathSegment';
import { IdlePathSegment } from '@fmgc/flightmanagement/vnav/segments/IdlePathSegment';
import { AltitudeConstraintType } from '@fmgc/guidance/lnav/legs';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { NodeContext } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

/**
 * TODO: In reality, this should use an idle path to determine which constraint is constraining and use a geometric path to this.
 * For now, we just use the first at-or-below constraint.
 */
export class ManagedDescentSegment extends ProfileSegment {
    /**
     * The reason this is an array is because we use one below the speed limit and one above the speed limit
     */
    private geometricSegments: GeometricPathSegment[] = [];

    private idleSegment: IdlePathSegment;

    constructor(context: NodeContext, private constraints: ConstraintReader) {
        super();

        const { managedDescentSpeed, cruiseAltitude, descentSpeedLimit } = context.observer.get();

        this.geometricSegments = [
            new GeometricPathSegment(context, constraints, managedDescentSpeed, cruiseAltitude),
        ];

        if (Number.isFinite(descentSpeedLimit.underAltitude) && Number.isFinite(descentSpeedLimit.speed)) {
            this.geometricSegments.unshift(
                new GeometricPathSegment(context, constraints, Math.min(descentSpeedLimit.speed, managedDescentSpeed), Math.min(descentSpeedLimit.underAltitude, cruiseAltitude)),
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

    private setGeometricPathPoint() {
        for (const constraint of this.constraints.descentAltitudeConstraints) {
            if (constraint.constraint.type !== AltitudeConstraintType.atOrAbove) {
                this.geometricSegments.forEach((segment) => segment.setLastConstraint(constraint));
            }
        }
    }
}
