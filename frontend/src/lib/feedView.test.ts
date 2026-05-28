import { describe, expect, it } from 'vitest';
import { transactionFieldLabel, transactionMetadataNotice } from './feedView';
import type { FeedTransactionSummary } from '../types';

describe('feed view helpers', () => {
  it('formats typed geometry-capable NOTAM field metadata', () => {
    const transaction = tx({
      notamType: 'NOTAMN',
      notamQCode: 'QRTCA',
      notamAffectedLocation: 'KZNY',
      notamHasGeometry: true,
      notamPermanentEnd: true
    });

    expect(transactionFieldLabel(transaction)).toBe('NOTAMN · QRTCA · KZNY · GEOM · PERM');
    expect(transactionMetadataNotice(transaction)).toContain('route-impact geometry');
  });

  it('labels non-geometric NOTAMs as retained constraints without map geometry', () => {
    const transaction = tx({
      notamType: 'NOTAMC',
      notamAffectedLocation: 'KZNY',
      notamHasGeometry: false,
      warnings: ['No compact coordinate/radius found in Q or E field.']
    });

    expect(transactionFieldLabel(transaction)).toBe('NOTAMC · KZNY · NO GEOM');
    expect(transactionMetadataNotice(transaction)).toContain('without map geometry');
  });

  it('uses accountability when affected location is absent and leaves non-NOTAM rows quiet', () => {
    expect(transactionFieldLabel(tx({ notamType: 'NOTAMJ', notamAccountability: 'CZYZ', notamEstimatedEnd: true })))
      .toBe('NOTAMJ · CZYZ · NO GEOM · EST');
    expect(transactionFieldLabel(tx({ type: 'SIGMET' }))).toBe('—');
    expect(transactionMetadataNotice(tx({ type: 'SIGMET' }))).toContain('No NOTAM');
  });

  it('formats domestic NOTAM semantic reducer metadata separately from ICAO fields', () => {
    const domestic = tx({
      type: 'DOMESTIC',
      domesticNotamKeyword: 'RWY',
      domesticNotamQ23: 'LE',
      domesticNotamQ45: 'XX',
      domesticNotamSemanticFacilityFamily: 'RWY',
      domesticNotamSemanticAction: 'CLOSED',
      domesticNotamReducerRuleId: 'DOM2.SURFACE.CLOSED'
    });
    const ambiguous = tx({
      type: 'DOMESTIC',
      domesticNotamKeyword: 'RAMP',
      domesticNotamReducerRuleId: 'DOM2.UNMATCHED'
    });

    expect(transactionFieldLabel(domestic)).toBe('RWY · RWY · CLOSED · DOM2.SURFACE.CLOSED · LE/XX');
    expect(transactionMetadataNotice(domestic)).toContain('DOM1 record metadata');
    expect(transactionMetadataNotice(ambiguous)).toContain('semantic reduction is ambiguous');
  });

  it('surfaces low-visibility and RVR domestic NOTAMs as procedure coordination constraints', () => {
    const rvr = tx({
      type: 'DOMESTIC',
      domesticNotamKeyword: 'SVC',
      domesticNotamQ23: 'FT',
      domesticNotamSemanticFacilityFamily: 'SVC',
      domesticNotamSemanticCondition: 'RVR',
      domesticNotamSemanticAction: 'UNAVAILABLE',
      domesticNotamReducerRuleId: 'DOM2.SVC.RVR'
    });
    const smgcs = tx({
      type: 'DOMESTIC',
      domesticNotamKeyword: 'SVC',
      domesticNotamSemanticFacilityFamily: 'SVC',
      domesticNotamSemanticCondition: 'LOW_VISIBILITY_PROCEDURE',
      domesticNotamReducerRuleId: 'DOM2.SVC.LOW_VISIBILITY_PROCEDURE'
    });

    expect(transactionFieldLabel(rvr)).toBe('SVC · SVC · RVR · DOM2.SVC.RVR · FT');
    expect(transactionMetadataNotice(rvr)).toContain('Low-visibility/RVR NOTAM');
    expect(transactionMetadataNotice(smgcs)).toContain('ICAO/operator terminology');
  });

  it('surfaces approach-minima and runway-friction domestic NOTAMs as safety constraints', () => {
    const approach = tx({
      type: 'DOMESTIC',
      domesticNotamKeyword: 'NAV',
      domesticNotamSemanticFacilityFamily: 'NAV',
      domesticNotamSemanticCondition: 'APPROACH_MINIMA',
      domesticNotamReducerRuleId: 'DOM2.NAV.APPROACH_MINIMA'
    });
    const friction = tx({
      type: 'DOMESTIC',
      domesticNotamKeyword: 'RWY',
      domesticNotamSemanticFacilityFamily: 'RWY',
      domesticNotamSemanticCondition: 'FRICTION',
      domesticNotamReducerRuleId: 'DOM2.SURFACE.FRICTION'
    });

    expect(transactionMetadataNotice(approach).toLowerCase()).toContain('approach/minima');
    expect(transactionMetadataNotice(friction).toLowerCase()).toContain('runway surface/friction');
  });

  it('formats service request and table commands separately from NOTAM constraints', () => {
    const request = tx({
      type: 'SERVICE_REQUEST',
      serviceCommandType: 'REQUEST',
      serviceCommandDomain: 'DOM',
      serviceCommandOperation: 'RQN',
      serviceCommandLocation: 'KJFK',
      serviceCommandHistory: true,
      serviceCommandPrivilegedHistoryRequest: true,
      serviceCommandAccepted: true
    });
    const table = tx({
      type: 'SERVICE_TABLE',
      serviceCommandType: 'TABLE',
      serviceCommandDomain: 'ROUTING',
      serviceCommandOperation: 'UPDATE',
      serviceCommandAccepted: true
    });

    expect(transactionFieldLabel(request)).toBe('REQUEST · DOM · RQN · KJFK · HISTORY/PRIV-HIST · ACCEPTED');
    expect(transactionMetadataNotice(request)).toContain('Service request command');
    expect(transactionFieldLabel(table)).toBe('TABLE · ROUTING · UPDATE · ACCEPTED');
    expect(transactionMetadataNotice(table)).toContain('table command');
  });

  it('formats retained legacy family metadata without treating it as NOTAM geometry', () => {
    const genot = tx({
      type: 'GENOT',
      familySemantic: 'genot-admin-message',
      familyLifecycle: 'admin-message',
      familyGenotSeries: 'RWA'
    });

    expect(transactionFieldLabel(genot)).toBe('GENOT · genot-admin-message · admin-message · RWA');
    expect(transactionMetadataNotice(genot)).toContain('retained as typed source-family metadata');
  });
});

function tx(overrides: Partial<FeedTransactionSummary>): FeedTransactionSummary {
  return {
    id: 'tx-1',
    type: 'ICAO_NOTAMN',
    status: 'ACCEPTED',
    supported: true,
    warnings: [],
    errors: [],
    ...overrides
  };
}
