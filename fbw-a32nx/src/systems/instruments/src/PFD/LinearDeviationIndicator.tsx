// Copyright (c) 2021-2023 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

import { Arinc429ConsumerSubject, ArincEventBus } from '@flybywiresim/fbw-sdk';
import { ConsumerSubject, DisplayComponent, FSComponent, MappedSubject, Subject, VNode } from '@microsoft/msfs-sdk';
import { FmsVars } from 'instruments/src/MsfsAvionicsCommon/providers/FmsDataPublisher';
import { Arinc429Values } from 'instruments/src/PFD/shared/ArincValueProvider';
import { FgBus } from './shared/FgBusProvider';
import { PFDSimvars } from './shared/PFDSimvarPublisher';
import { getDisplayIndex } from './PFD';

type LinearDeviationIndicatorProps = {
  bus: ArincEventBus;
};

export class LinearDeviationIndicator extends DisplayComponent<LinearDeviationIndicatorProps> {
  private readonly altitude = Arinc429ConsumerSubject.create(
    this.props.bus.getArincSubscriber<Arinc429Values>().on('altitudeAr'),
  );

  private readonly linearDeviationRequested = ConsumerSubject.create(
    this.props.bus.getSubscriber<FmsVars>().on('linearDeviationActive'),
    false,
  );

  private componentTransform = Subject.create('');

  private upperLinearDeviationReadoutText = Subject.create('00');

  private upperLinearDeviationReadoutVisibility = Subject.create<'visible' | 'hidden'>('hidden');

  private lowerLinearDeviationReadoutText = Subject.create('00');

  private lowerLinearDeviationReadoutVisibility = Subject.create<'visible' | 'hidden'>('hidden');

  private linearDeviationDotVisibility = Subject.create<'visible' | 'hidden'>('hidden');

  private linearDeviationDotUpperHalfVisibility = Subject.create<'visible' | 'hidden'>('hidden');

  private linearDeviationDotLowerHalfVisibility = Subject.create<'visible' | 'hidden'>('hidden');

  private latchSymbolVisibility = Subject.create<'visible' | 'hidden'>('hidden');

  // TODO: Use ARINC value for this
  private readonly targetAltitude = ConsumerSubject.create(
    this.props.bus
      .getSubscriber<FmsVars>()
      .on('targetAltitude')
      .atFrequency(1000 / 60),
    undefined,
  );

  private readonly fmgcDiscreteWord1 = Arinc429ConsumerSubject.create(
    this.props.bus.getArincSubscriber<FgBus>().on('fmgcDiscreteWord1'),
  );

  private readonly fmgcDiscreteWord2 = Arinc429ConsumerSubject.create(
    this.props.bus.getArincSubscriber<FgBus>().on('fmgcDiscreteWord2'),
  );

  private readonly fmgcDiscreteWord3 = Arinc429ConsumerSubject.create(
    this.props.bus.getArincSubscriber<FgBus>().on('fmgcDiscreteWord3'),
  );

  private readonly finalArmedOrActive = MappedSubject.create(
    ([discrete1, discrete2, discrete3]) =>
      discrete3.bitValueOr(23, false) || // FINAL armed
      (discrete2.bitValueOr(12, false) && discrete1.bitValueOr(23, false)), // FINAL APP active
    this.fmgcDiscreteWord1,
    this.fmgcDiscreteWord2,
    this.fmgcDiscreteWord3,
  );

  private readonly vdevRequest = ConsumerSubject.create(null, false);

  private readonly linearDeviationInhibited = MappedSubject.create(
    ([linearDeviationActive, finalArmedOrActive, vdevRequest]) =>
      !linearDeviationActive || finalArmedOrActive || vdevRequest,
    this.linearDeviationRequested,
    this.finalArmedOrActive,
    this.vdevRequest,
  );

  onAfterRender(node: VNode): void {
    super.onAfterRender(node);

    const sub = this.props.bus.getSubscriber<Arinc429Values & FmsVars & PFDSimvars>();

    const onAltChangedSub = this.altitude.sub((alt) => {
      const targetAlt = this.targetAltitude.get();
      if (!alt.isNormalOperation() || targetAlt === undefined) {
        this.hide();
        return;
      }

      const deviation = alt.value - targetAlt;
      const pixelOffset = this.pixelOffsetFromDeviation(Math.max(Math.min(deviation, 500), -500));

      this.componentTransform.set(`translate(0 ${pixelOffset})`);

      const linearDeviationReadoutText = Math.min(99, Math.round(Math.abs(deviation) / 100))
        .toFixed(0)
        .padStart(2, '0');

      if (this.upperLinearDeviationReadoutVisibility.get() === 'visible') {
        this.upperLinearDeviationReadoutText.set(linearDeviationReadoutText);
      }

      if (this.lowerLinearDeviationReadoutVisibility.get() === 'visible') {
        this.lowerLinearDeviationReadoutText.set(linearDeviationReadoutText);
      }

      if (deviation > 540) {
        this.lowerLinearDeviationReadoutVisibility.set('visible');
        this.linearDeviationDotLowerHalfVisibility.set('visible');

        this.upperLinearDeviationReadoutVisibility.set('hidden');
        this.linearDeviationDotUpperHalfVisibility.set('hidden');

        this.linearDeviationDotVisibility.set('hidden');
      } else if (deviation > -500 && deviation < 500) {
        this.lowerLinearDeviationReadoutVisibility.set('hidden');
        this.linearDeviationDotLowerHalfVisibility.set('hidden');

        this.upperLinearDeviationReadoutVisibility.set('hidden');
        this.linearDeviationDotUpperHalfVisibility.set('hidden');

        this.linearDeviationDotVisibility.set('visible');
      } else if (deviation < -540) {
        this.lowerLinearDeviationReadoutVisibility.set('hidden');
        this.linearDeviationDotLowerHalfVisibility.set('hidden');

        this.upperLinearDeviationReadoutVisibility.set('visible');
        this.linearDeviationDotUpperHalfVisibility.set('visible');

        this.linearDeviationDotVisibility.set('hidden');
      }
    }, true);

    sub
      .on('verticalProfileLatched')
      .whenChanged()
      .handle((s) => this.latchSymbolVisibility.set(s ? 'visible' : 'hidden'));

    this.vdevRequest.setConsumer(sub.on(getDisplayIndex() === 1 ? 'vdevRequestLeft' : 'vdevRequestRight'));

    this.linearDeviationInhibited.sub((isInhibited) => {
      if (isInhibited) {
        this.hide();
        onAltChangedSub.pause();
      } else {
        onAltChangedSub.resume();
      }
    }, true);
  }

  private hide() {
    this.upperLinearDeviationReadoutVisibility.set('hidden');
    this.lowerLinearDeviationReadoutVisibility.set('hidden');
    this.linearDeviationDotLowerHalfVisibility.set('hidden');
    this.linearDeviationDotUpperHalfVisibility.set('hidden');
    this.linearDeviationDotVisibility.set('hidden');
  }

  render(): VNode {
    return (
      <g id="LinearDeviationIndicator">
        <text visibility={this.upperLinearDeviationReadoutVisibility} x="110" y="42.5" class="FontSmallest Green">
          {this.upperLinearDeviationReadoutText}
        </text>
        <g id="LinearDeviationDot" transform={this.componentTransform}>
          <path
            id="EntireDot"
            visibility={this.linearDeviationDotVisibility}
            d="m119.26 80.796a1.511 1.5119 0 1 0-3.022 0 1.511 1.5119 0 1 0 3.022 0z"
            class="Fill Green"
          />
          <path
            id="DotUpperHalf"
            visibility={this.linearDeviationDotUpperHalfVisibility}
            d="m116.24 80.796c4.9e-4 -0.83466 0.67686-1.511 1.511-1.511 0.83418 0 1.5105 0.67635 1.511 1.511h-1.511z"
            class="Fill Green"
          />
          <path
            id="DotLowerHalf"
            visibility={this.linearDeviationDotLowerHalfVisibility}
            d="m116.24 80.796c4.9e-4 0.83465 0.67686 1.511 1.511 1.511 0.83418 0 1.5105-0.67636 1.511-1.511h-1.511z"
            class="Fill Green"
          />
          <path visibility={this.latchSymbolVisibility} d="m 119 78.3 h -3 v 5 h 3" class="Magenta" />
        </g>
        <text visibility={this.lowerLinearDeviationReadoutVisibility} x="110" y="123" class="FontSmallest Green">
          {this.lowerLinearDeviationReadoutText}
        </text>
      </g>
    );
  }

  private pixelOffsetFromDeviation(deviation: number) {
    return (deviation * 40.5) / 500;
  }
}
