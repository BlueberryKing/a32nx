import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { ManagedClimbMachSegment } from '@fmgc/flightmanagement/vnav/segments/ManagedClimbMachSegment';
import { ManagedClimbSegment } from '@fmgc/flightmanagement/vnav/segments/ManagedClimbSegment';
import { SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { FmgcFlightPhase } from '@shared/flightphase';
import { McduPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';

export class ClimbSegment extends ProfileSegment {
    constructor(context: SegmentContext, constraints: ConstraintReader) {
        super();

        const { cruiseAltitude, climbSpeedLimit, managedClimbSpeed, managedClimbSpeedMach } = context.observer.get();
        const crossoverAltitude = context.atmosphericConditions.crossoverAltitude(managedClimbSpeed, managedClimbSpeedMach);

        this.children = [
            new ManagedClimbSegment(context, climbSpeedLimit.speed, Math.min(climbSpeedLimit.underAltitude, cruiseAltitude, crossoverAltitude), constraints),
            new ManagedClimbSegment(context, managedClimbSpeed, Math.min(crossoverAltitude, cruiseAltitude), constraints),
            new ManagedClimbMachSegment(context, cruiseAltitude, managedClimbSpeedMach),
        ];
    }

    onAfterBuildingChildren(builder: ProfileBuilder): void {
        builder.requestPseudoWaypoint(McduPseudoWaypointType.TopOfClimb, builder.lastState);
        builder.changePhase(FmgcFlightPhase.Cruise);
    }

    get repr() {
        return 'ClimbSegment';
    }
}
