import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { SegmentContext } from '@fmgc/flightmanagement/vnav/segments';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureIdlePathConstantMachSegment } from '@fmgc/flightmanagement/vnav/segments/descent/PureIdlePathConstantMachSegment';
import { PropagatorOptions } from '@fmgc/flightmanagement/vnav/integrators';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';
import { IdlePathToAltitudeSegment } from './IdlePathToAltitudeSegment';

export class IdlePathSegment extends ProfileSegment {
    constructor(context: SegmentContext, constraints: ConstraintReader, crossoverAltitude: Feet, toAltitude: Feet) {
        super();

        const { descentSpeedLimit, managedDescentSpeed } = context.observer.get();

        const options: PropagatorOptions = {
            stepSize: -5,
            windProfileType: WindProfileType.Descent,
            useMachVsCas: false,
        };

        this.children = [
            new IdlePathToAltitudeSegment(
                context, constraints, Math.min(descentSpeedLimit.underAltitude, crossoverAltitude, toAltitude), descentSpeedLimit.speed, options,
            ),
            new IdlePathToAltitudeSegment(
                context, constraints, Math.min(crossoverAltitude, toAltitude), managedDescentSpeed, options,
            ),
            new PureIdlePathConstantMachSegment(context, toAltitude, -Infinity),
        ];
    }

    get repr(): string {
        return 'IdlePathSegment';
    }
}
