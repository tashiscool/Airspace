package org.tash.extensions.evaluation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AirspaceEvaluationLedger {
    private final List<ArtifactEvaluationEntry> entries = new ArrayList<>();

    public AirspaceEvaluationLedger add(ArtifactEvaluationEntry entry) {
        entries.add(entry);
        return this;
    }

    public List<ArtifactEvaluationEntry> entries() {
        return Collections.unmodifiableList(entries);
    }

    public long count(ArtifactEvaluationStatus status) {
        return entries.stream().filter(entry -> entry.getStatus() == status).count();
    }

    public boolean hasUnreviewedEntries() {
        return count(ArtifactEvaluationStatus.UNREVIEWED) > 0;
    }

    public static AirspaceEvaluationLedger defaultBacklog() {
        AirspaceEvaluationLedger ledger = new AirspaceEvaluationLedger();
        ledger.add(entry("~/Downloads/tworessies", ArtifactEvaluationStatus.TEST_ADDED, "CARF conflict regression",
                "Named fixes, ADMIS MITO, seconds admission, and conflict parsing are covered."));
        ledger.add(entry("~/Downloads/runtheseandseewhyconflict", ArtifactEvaluationStatus.TEST_ADDED, "CARF head-on regression",
                "Head-on route conflict explanation tests are covered."));
        ledger.add(entry("~/Downloads/attachments/HEADON ONE.txt", ArtifactEvaluationStatus.TEST_ADDED, "CARF head-on regression",
                "PTR 045 no-climb attachment is parsed and tested."));
        ledger.add(entry("~/Downloads/attachments/HEADON TWO.txt", ArtifactEvaluationStatus.TEST_ADDED, "CARF head-on regression",
                "PTR 045 no-climb attachment is parsed and tested."));
        ledger.add(entry("~/Downloads/conflicttoshort", ArtifactEvaluationStatus.TEST_ADDED, "CARF duration regression",
                "Minimum-duration filtering is explicit and tested."));
        ledger.add(entry("~/Downloads/fwdconflicttoshort", ArtifactEvaluationStatus.TEST_ADDED, "CARF duration regression",
                "Forwarded duplicate listing and screenshot are classified."));
        ledger.add(entry("~/Downloads/fwdconflicttoshort duplicate comparison", ArtifactEvaluationStatus.REFERENCE_ONLY, "CARF duration regression",
                "Byte/hash duplicate of original short-conflict evidence."));
        ledger.add(entry("~/Downloads/messages.eml", ArtifactEvaluationStatus.TEST_ADDED, "CARF timing regression",
                "Email attachment inventory and same-year timing conflicts are covered."));
        ledger.add(entry("~/Downloads/messages.zip", ArtifactEvaluationStatus.TEST_ADDED, "CARF timing regression",
                "Timing fixtures parse directly from ZIP."));
        ledger.add(entry("~/Downloads/texts", ArtifactEvaluationStatus.TEST_ADDED, "CARF timing regression",
                "Plain timing fixtures are classified and covered."));
        ledger.add(entry("~/Downloads/ptr-02-*", ArtifactEvaluationStatus.TEST_ADDED, "CARF longitudinal regression",
                "PTR 002 opposite-direction no-false-conflict behavior is covered."));
        ledger.add(entry("~/Downloads/mrkoci", ArtifactEvaluationStatus.TEST_ADDED, "CARF radial/DME regression",
                "Radial/DME pair parses and exact-geometry conflicts are covered."));
        ledger.add(entry("~/Downloads/reenjoiitaly___buttheresmorecarfwork.zip", ArtifactEvaluationStatus.IMPLEMENTED, "Legacy spatial model",
                "Reusable algorithms were modernized; Luciad/JUNG dependencies remain reference-only."));
        ledger.add(entry("~/Downloads/Dom1.ycc", ArtifactEvaluationStatus.TEST_ADDED, "DOM parser",
                "DOM1 structure, dates, comments, cancellations, edits, and malformed WEF behavior are tested."));
        ledger.add(entry("~/Downloads/Dom2.ycc", ArtifactEvaluationStatus.IMPLEMENTED, "DOM2 parser",
                "Grammar-backed contraction dictionary and selected q-code reducer behavior are implemented."));
        ledger.add(entry("~/Downloads/newDom2.ycc", ArtifactEvaluationStatus.IMPLEMENTED, "DOM2 parser",
                "Diff-added contractions are covered."));
        ledger.add(entry("~/Downloads/domparse2.ycc", ArtifactEvaluationStatus.REFERENCE_ONLY, "DOM2 parser",
                "Original workspace grammar is retained as reference; modern parser owns behavior."));
        ledger.add(entry("~/Downloads/fwdfwdkeywordsthatneedclarification", ArtifactEvaluationStatus.TEST_ADDED, "DOM2 keyword clarification",
                "Representative rows from all 14 spreadsheets are parsed and classified."));
        ledger.add(entry("~/Downloads/Visual Parser Files.eml", ArtifactEvaluationStatus.TEST_ADDED, "Parser tester archives",
                "Attachments and Visual Parser workspaces are inventoried."));
        ledger.add(entry("~/Downloads/Big Zip file for parser tester...and that's what she said.eml", ArtifactEvaluationStatus.REFERENCE_ONLY, "Parser tester archives",
                "WebSphere client archive is vendor/runtime baggage."));
        ledger.add(entry("~/Downloads/Bundle.eml", ArtifactEvaluationStatus.TEST_ADDED, "Parser tester archives",
                "Nested parser tester workspaces contribute fixtures."));
        ledger.add(entry("~/Downloads/navaids", ArtifactEvaluationStatus.IMPLEMENTED, "Waypoint resolution",
                "Resolver diagnostics, composite resolver, file resolver, and navaid route fixtures are covered."));
        ledger.add(entry("~/Downloads/GLOBALACCOUNTS.xls", ArtifactEvaluationStatus.IMPLEMENTED, "DOM account defaults",
                "Global-account keyword resolver is implemented and tested."));
        ledger.add(entry("~/Downloads/GLOBALACCOUNTSreal.xls", ArtifactEvaluationStatus.REFERENCE_ONLY, "DOM account defaults",
                "Duplicate/reference account sheet."));
        ledger.add(entry("~/Downloads/FIRs.xls", ArtifactEvaluationStatus.IMPLEMENTED, "FIR reference data",
                "Identifier/description catalog extraction is implemented."));
        ledger.add(entry("~/Downloads/CARF Fax Number.xls", ArtifactEvaluationStatus.REFERENCE_ONLY, "CARF reference data",
                "Contact metadata only; no executable CARF rows."));
        ledger.add(entry("~/Downloads/carftesting.xls", ArtifactEvaluationStatus.REFERENCE_ONLY, "CARF reference data",
                "Legacy workbook has no extractable executable rows with local tooling."));
        ledger.add(entry("~/Downloads/AEROBAT/BERMS/PILE/REMAINING/TURNAROUND.xls", ArtifactEvaluationStatus.TEST_ADDED, "Domestic NOTAM spreadsheets",
                "Executable rows feed airspace, surface-condition, runway-equipment, and laser parsers."));
        ledger.add(entry("~/Downloads/CLEANCARF.backup", ArtifactEvaluationStatus.BLOCKED, "Postgres reference dump",
                "Custom dump is inventoried; row extraction needs pg_restore."));
        ledger.add(entry("~/Downloads/iRD - Protected Airspace Rules for CARF.doc", ArtifactEvaluationStatus.IMPLEMENTED, "CARF rules",
                "Core rules are represented by CarfProtectedAirspaceRules and route geometry tests."));
        ledger.add(entry("~/Downloads/autodrawFiles", ArtifactEvaluationStatus.REFERENCE_ONLY, "Display geometry",
                "Legacy Luciad/UI code is dependency-heavy; duplicate-navaid and geometry behavior was ported."));
        ledger.add(entry("~/Downloads/carf-checkin.png", ArtifactEvaluationStatus.REFERENCE_ONLY, "Visual evidence",
                "Screenshot dimensions/hash are inventoried."));
        ledger.add(entry("~/Downloads/ptr-02-original-conflict.png", ArtifactEvaluationStatus.REFERENCE_ONLY, "Visual evidence",
                "Old display symptom screenshot is inventoried."));
        ledger.add(entry("~/Downloads/ptr-02-now-conflict.png", ArtifactEvaluationStatus.REFERENCE_ONLY, "Visual evidence",
                "Corrected display screenshot is inventoried."));
        ledger.add(entry("~/Downloads/conflict display.eml", ArtifactEvaluationStatus.BLOCKED, "Encrypted display archive",
                "Encrypted RAR headers require password/tooling."));
        ledger.add(entry("/Users/tkhan/Downloads/src 2 messaging", ArtifactEvaluationStatus.IMPLEMENTED, "USNS messaging/classification",
                "Modern MRS/NADIN/WMSCR envelope parsing, transaction classification, policy outcomes, family semantics, access checks, ingest facade, and sample fixture expansion are covered."));
        ledger.add(entry("/Users/tkhan/Downloads/src 2 infrastructure", ArtifactEvaluationStatus.REFERENCE_ONLY, "USNS legacy infrastructure",
                "EJB, DAO, WebSphere, JDBC, and generated parser tables remain reference-only and are not copied into production."));
        ledger.add(entry("/Users/tkhan/Downloads/src 3/gov/faa/aim/carf/parser/ALTRV.g", ArtifactEvaluationStatus.IMPLEMENTED, "CARF ALTRV parser",
                "Modern ALTRV tokenizer/parser, feature diagnostics, route graph validation, and reservation mapper use the grammar as coverage reference."));
        ledger.add(entry("~/Downloads/CARF Hibernate configurations.docx", ArtifactEvaluationStatus.IMPLEMENTED, "CARF persistence/lifecycle",
                "Mission mapping and Scarf lifecycle notes are represented by schema catalog and pure lifecycle service."));
        ledger.add(entry("~/Downloads/training.backup", ArtifactEvaluationStatus.REFERENCE_ONLY, "CARF PostgreSQL schema",
                "Core tables are cataloged for future row extraction; live database restore remains adapter-backed."));
        return ledger;
    }

    private static ArtifactEvaluationEntry entry(String path,
                                                 ArtifactEvaluationStatus status,
                                                 String category,
                                                 String finding) {
        return ArtifactEvaluationEntry.builder()
                .path(path)
                .status(status)
                .category(category)
                .finding(finding)
                .testReference("")
                .build();
    }
}
