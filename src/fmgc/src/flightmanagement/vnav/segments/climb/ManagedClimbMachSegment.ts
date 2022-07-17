import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { PureAccelerationSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureAccelerationSegment';
import { PureClimbToAltitudeSegment } from '@fmgc/flightmanagement/vnav/segments/climb/PureClimbToAltitudeSegment';
import { ClimbProfileRequest } from '@fmgc/flightmanagement/vnav/ClimbProfileRequest';

export class ManagedClimbMachSegment extends ProfileSegment {
    constructor(request: ClimbProfileRequest, toAltitude: Feet, maxSpeed: Knots, maxMach: Mach) {
        super();

        this.children = [
            PureAccelerationSegment.toMach(request.accelerationPropagator, maxSpeed, maxMach, toAltitude),
            new PureClimbToAltitudeSegment(request.climbPropagator, toAltitude),
        ];
    }

    get repr() {
        return 'ManagedClimbMachSegment';
    }
}
