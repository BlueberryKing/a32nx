import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { ManagedClimbMachSegment } from '@fmgc/flightmanagement/vnav/segments/ManagedClimbMachSegment';
import { ManagedClimbSegment } from '@fmgc/flightmanagement/vnav/segments/ManagedClimbSegment';
import { NodeContext, SymbolBuilder } from '@fmgc/flightmanagement/vnav/segments/index';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';

export class ClimbSegment extends ProfileSegment {
    constructor(context: NodeContext, constraints: ConstraintReader) {
        super();

        const { cruiseAltitude, climbSpeedLimit, managedClimbSpeed, managedClimbSpeedMach } = context.observer.get();
        const crossoverAltitude = 30000; // TODO

        this.children = [
            new ManagedClimbSegment(context, climbSpeedLimit.speed, climbSpeedLimit.underAltitude, constraints),
            new ManagedClimbSegment(context, managedClimbSpeed, crossoverAltitude, constraints),
            new ManagedClimbMachSegment(cruiseAltitude, managedClimbSpeedMach),
        ];
    }

    getSymbols(symbolBuilder: SymbolBuilder): void {
        symbolBuilder.push();
    }

    get repr() {
        return 'ClimbNode';
    }
}
