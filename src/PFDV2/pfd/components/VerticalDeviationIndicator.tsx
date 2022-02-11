import { DisplayComponent, VNode } from 'msfssdk';

type VerticalDeviationIndicatorProps = {
    deviation: number
}

export class VerticalDeviationIndicator extends DisplayComponent<VerticalDeviationIndicatorProps> {
    onAfterRender(node: VNode): void {
        super.onAfterRender(node);
    }

    render(): VNode {
        const pixelOffset = this.pixelOffsetFromDeviation(this.props.deviation);

        return (
            <g>
                <circle cx="117" cy={80.8 + pixelOffset} r="1" className="Fill Green" />
            </g>
        );
    }

    private pixelOffsetFromDeviation(deviation: number) {
        return deviation * 20 / 500;
    }
}
