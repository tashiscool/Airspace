package org.tash.extensions.notam.access;

public interface NotamAccessReferenceData {
    boolean hasAccountPrivilege(String privilege, String accountId, String series, String source);

    boolean hasLocationPrivilege(String privilege, String locationId, String series, String source);

    boolean hasAnyDomesticPrivilege(String privilege);

    boolean hasAnyInternationalPrivilege(String privilege);

    boolean hasAnyMilitaryPrivilege(String privilege);
}
