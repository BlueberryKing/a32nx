import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { ManagedClimbMachSegment } from '@fmgc/flightmanagement/vnav/segments/climb/ManagedClimbMachSegment';
import { ManagedClimbSegment } from '@fmgc/flightmanagement/vnav/segments/climb/ManagedClimbSegment';
import { SegmentContext, ProfileBuilder, AircraftState } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { FmgcFlightPhase } from '@shared/flightphase';
import { McduPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';

export class ClimbSegment extends ProfileSegment {
    constructor(private context: SegmentContext, constraints: ConstraintReader) {
        super();

        const { cruiseAltitude, climbSpeedLimit, managedClimbSpeed, managedClimbSpeedMach } = context.observer.get();
        const crossoverAltitude = context.computeCrossoverAltitude(managedClimbSpeed, managedClimbSpeedMach);

        this.children = [
            new ManagedClimbSegment(context, climbSpeedLimit.speed, Math.min(climbSpeedLimit.underAltitude, cruiseAltitude, crossoverAltitude), constraints),
            new ManagedClimbSegment(context, managedClimbSpeed, Math.min(crossoverAltitude, cruiseAltitude), constraints),
            new ManagedClimbMachSegment(context, cruiseAltitude, managedClimbSpeedMach),
        ];
    }

    shouldCompute(_state: AircraftState, _builder: ProfileBuilder): boolean {
        return this.context.observer.get().flightPhase <= FmgcFlightPhase.Climb;
    }

    compute(_state: AircraftState, builder: ProfileBuilder): void {
        builder.changePhase(FmgcFlightPhase.Climb);
    }

    onAfterBuildingChildren(builder: ProfileBuilder): void {
        builder.requestPseudoWaypoint(McduPseudoWaypointType.TopOfClimb, builder.lastState);
    }

    get repr() {
        return 'ClimbSegment';
    }
}
