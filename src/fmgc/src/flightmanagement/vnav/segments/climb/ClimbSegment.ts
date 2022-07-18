import { ManagedClimbMachSegment } from '@fmgc/flightmanagement/vnav/segments/climb/ManagedClimbMachSegment';
import { ManagedClimbSegment } from '@fmgc/flightmanagement/vnav/segments/climb/ManagedClimbSegment';
import { SegmentContext, ProfileBuilder, AircraftState } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { FmgcFlightPhase } from '@shared/flightphase';
import { McduPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';
import { ClimbProfileRequest } from '@fmgc/flightmanagement/vnav/ClimbProfileRequest';

export class ClimbSegment extends ProfileSegment {
    constructor(private context: SegmentContext, request: ClimbProfileRequest) {
        super();

        const { cruiseAltitude } = context.observer.get();
        const crossoverAltitude = context.computeCrossoverAltitude(request.maxSpeed, request.maxMach);

        // If we want to consider the speed limit in this profile, we insert a segment before the main segment.
        if (request.speedLimit !== null) {
            this.children.push(
                new ManagedClimbSegment(request, context, Math.min(request.speedLimit.speed, request.maxSpeed), Math.min(request.speedLimit.underAltitude, cruiseAltitude, crossoverAltitude)),
            );
        }

        this.children.push(new ManagedClimbSegment(request, context, request.maxSpeed, Math.min(crossoverAltitude, cruiseAltitude)));
        this.children.push(new ManagedClimbMachSegment(request, cruiseAltitude, request.maxSpeed, request.maxMach));
    }

    shouldCompute(_state: AircraftState, _builder: ProfileBuilder): boolean {
        return this.context.observer.get().flightPhase <= FmgcFlightPhase.Climb;
    }

    compute(_state: AircraftState, builder: ProfileBuilder): void {
        builder.changePhase(FmgcFlightPhase.Climb);
    }

    onAfterBuildingChildren(builder: ProfileBuilder): void {
        builder.requestMcduPseudoWaypoint(McduPseudoWaypointType.TopOfClimb, builder.lastState);
    }

    get repr() {
        return 'ClimbSegment';
    }
}
