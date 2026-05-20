package org.tash.extensions.feed;

/**
 * Adapter seam for authoritative navaid/fix/aerodrome/account reference data sync.
 */
public interface ReferenceDataSyncAdapter {
    ReferenceDataSyncResult preview(String payload);
}
