import { describe, expect, it } from 'vitest';
import { searchTypeLabel, selectionForSearch } from './searchView';

describe('search view helpers', () => {
  it('labels feed transactions and preserves the feed artifact selection id', () => {
    const selection = selectionForSearch({
      id: 'artifact-1#artifact-1#0',
      type: 'feed-transaction',
      title: 'DOM2.SURFACE.CLOSED',
      status: 'ACCEPTED',
      snippet: 'RWY RWY CLOSED DOM2.SURFACE.CLOSED',
      route: '/feed/artifact-1'
    });

    expect(searchTypeLabel('feed-transaction')).toBe('Feed transaction');
    expect(selection).toMatchObject({
      feedArtifactId: 'artifact-1',
      sourceFamily: 'NOTAM',
      label: 'Feed transaction: DOM2.SURFACE.CLOSED'
    });
    expect(selection.messageId).toBeUndefined();
  });

  it('keeps generic feed hits as USNS traffic and weather hits as weather', () => {
    expect(selectionForSearch({ id: 'feed-1', type: 'feed', title: 'operator USNS', snippet: 'raw traffic' }))
      .toMatchObject({ feedArtifactId: 'feed-1', sourceFamily: 'USNS' });
    expect(selectionForSearch({ id: 'feed-2#feed-2#1', type: 'feed-transaction', title: 'SIGMET', snippet: 'SIGMET line' }))
      .toMatchObject({ feedArtifactId: 'feed-2', sourceFamily: 'WEATHER' });
  });
});
