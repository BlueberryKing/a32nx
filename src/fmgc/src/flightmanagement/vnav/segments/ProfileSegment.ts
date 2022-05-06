import { AircraftState, ProfileBuilder, Visitor, VisitorContext, VerticalSegmentType } from '@fmgc/flightmanagement/vnav/segments';

export abstract class ProfileSegment {
    protected children: ProfileSegment[] = [];

    compute(_state: AircraftState, _builder: ProfileBuilder) { }

    accept(visitor: Visitor, context: VisitorContext = { depth: 0 }) {
        visitor.visitBeforeChildren(this, context);

        for (const child of this.children) {
            child.accept(visitor, { depth: context.depth + 1 });
        }

        visitor.visitAfterChildren(this, context);
    }

    getSymbols(_symbolBuilder: any) { }

    get repr(): string {
        return 'Unknown node';
    }

    get type(): VerticalSegmentType {
        return VerticalSegmentType.Unknown;
    }
}
