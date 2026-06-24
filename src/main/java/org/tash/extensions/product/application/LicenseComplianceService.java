package org.tash.extensions.product.application;

import jakarta.enterprise.context.ApplicationScoped;
import org.eclipse.microprofile.config.inject.ConfigProperty;
import org.tash.extensions.product.dto.ComplianceDtos;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.CopyOnWriteArrayList;

@ApplicationScoped
public class LicenseComplianceService {
    public static final String POLICY_VERSION = "airspace-public-use-ai-transparency-v1";

    @ConfigProperty(name = "airspace.compliance.repository-url", defaultValue = "https://github.com/tashiscool/Airspace")
    String repositoryUrl;

    @ConfigProperty(name = "airspace.compliance.collector-url", defaultValue = "UNCONFIGURED")
    String collectorUrl;

    @ConfigProperty(name = "airspace.compliance.public-registry-url", defaultValue = "https://github.com/tashiscool/Airspace/issues/new?template=airspace-use-disclosure.yml")
    String publicRegistryUrl;

    private final CopyOnWriteArrayList<ComplianceDtos.ComplianceAttestationSummary> attestations = new CopyOnWriteArrayList<>();

    public ComplianceDtos.CompliancePolicySummary policy() {
        return ComplianceDtos.CompliancePolicySummary.builder()
                .policyVersion(POLICY_VERSION)
                .licenseSpdx("AGPL-3.0-or-later")
                .legalLicense("GNU Affero General Public License v3.0 or later")
                .collectorMode("EXPLICIT_OPT_IN_ATTESTATION")
                .hiddenTelemetryEnabled(false)
                .cloneTimeCollectionEnabled(false)
                .sourceDisclosureRequired(true)
                .operationalConfigDisclosureRequired(true)
                .secretRedactionRequired(true)
                .obligations(List.of(
                        "Publish the complete corresponding source for modifications, network-service deployments, ports, and LLM-assisted recreations.",
                        "Publish safety-relevant decision rules, parsers, scenarios, prompts, policy logic, traces, and evaluation fixtures derived from Airspace.",
                        "Publish non-secret operational configuration needed to inspect how the implementation runs.",
                        "Clearly disclose use of Airspace and link back to the upstream repository.",
                        "Preserve downstream access to source and transparency obligations."))
                .prohibitedCollection(List.of(
                        "No hidden telemetry.",
                        "No clone-time beacon or automatic network call.",
                        "No collection of secrets, credentials, tokens, private operational data, or personal data.",
                        "No external submission unless an operator explicitly configures and runs it."))
                .documents(List.of(
                        "LICENSE.md",
                        "PUBLIC_USE_AND_AI_POLICY.md",
                        "COMPLIANCE.md",
                        "docs/COMPLIANCE_COLLECTOR.md"))
                .legalNote("AGPL-3.0-or-later is the legal license. Broader disclosure expectations for independent recreation or operational configurations are project policy and may require lawyer-drafted custom terms for strict enforceability.")
                .build();
    }

    public ComplianceDtos.ComplianceManifest manifest() {
        String trimmedCollector = "UNCONFIGURED".equals(trim(collectorUrl)) ? "" : trim(collectorUrl);
        return ComplianceDtos.ComplianceManifest.builder()
                .projectName("Airspace")
                .projectVersion("1.0-SNAPSHOT")
                .policyVersion(POLICY_VERSION)
                .repositoryUrl(repositoryUrl)
                .licenseSpdx("AGPL-3.0-or-later")
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .hiddenTelemetryEnabled(false)
                .cloneTimeCollectionEnabled(false)
                .sourceDisclosureRequired(true)
                .operationalConfigDisclosureRequired(true)
                .secretRedactionRequired(true)
                .collectorUrlConfigured(!trimmedCollector.isBlank())
                .collectorUrl(trimmedCollector.isBlank() ? null : trimmedCollector)
                .attestationEndpoint("/api/compliance/attestations")
                .publicRegistryUrl(publicRegistryUrl)
                .requiredDisclosureFields(List.of(
                        "organization",
                        "purpose",
                        "useType",
                        "publicDisclosureUrl",
                        "sourceCodeUrl",
                        "operationalConfigUrl",
                        "acknowledgesAgpl",
                        "acknowledgesPublicUseAndAiPolicy",
                        "sourcePublished",
                        "operationalConfigPublished",
                        "secretsRedacted"))
                .nonSecretOperationalConfigFields(List.of(
                        "deployment topology",
                        "enabled adapters and provider modes",
                        "rule catalog version",
                        "calibration model version",
                        "decision thresholds",
                        "agent policy version",
                        "audit/replay settings",
                        "source-freshness policy",
                        "redacted environment variable names without values"))
                .redactionRules(List.of(
                        "Never publish secrets, credentials, tokens, API keys, private certificates, passwords, or live operational data.",
                        "Publish redacted configuration templates and policy values needed for safety review.",
                        "Replace secret values with <REDACTED> and document how the value is used.",
                        "Do not include personal data or sensitive aviation-security details in public attestations."))
                .diagnostics(List.of(
                        "Git clone cannot execute a safe collector; Airspace uses explicit opt-in attestation instead.",
                        "This instance does not make any automatic compliance network call."))
                .build();
    }

    public ComplianceDtos.ComplianceAttestationSummary attest(ComplianceDtos.ComplianceAttestationRequest request) {
        ComplianceDtos.ComplianceAttestationRequest safe = request == null
                ? new ComplianceDtos.ComplianceAttestationRequest()
                : request;
        List<String> diagnostics = diagnostics(safe);
        boolean accepted = diagnostics.isEmpty();
        ComplianceDtos.ComplianceAttestationSummary summary = ComplianceDtos.ComplianceAttestationSummary.builder()
                .id("attestation-" + sha256(canonical(safe)).substring(0, 16))
                .accepted(accepted)
                .status(accepted ? "ACCEPTED_PUBLIC_DISCLOSURE" : "INCOMPLETE_DISCLOSURE")
                .policyVersion(POLICY_VERSION)
                .receivedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .organization(trim(safe.getOrganization()))
                .contact(trim(safe.getContact()))
                .projectUrl(trim(safe.getProjectUrl()))
                .sourceCodeUrl(trim(safe.getSourceCodeUrl()))
                .operationalConfigUrl(trim(safe.getOperationalConfigUrl()))
                .publicDisclosureUrl(trim(safe.getPublicDisclosureUrl()))
                .purpose(trim(safe.getPurpose()))
                .useType(trim(safe.getUseType()))
                .deploymentType(trim(safe.getDeploymentType()))
                .actor(trim(safe.getActor()))
                .acknowledgesAgpl(safe.isAcknowledgesAgpl())
                .acknowledgesPublicUseAndAiPolicy(safe.isAcknowledgesPublicUseAndAiPolicy())
                .sourcePublished(safe.isSourcePublished())
                .operationalConfigPublished(safe.isOperationalConfigPublished())
                .secretsRedacted(safe.isSecretsRedacted())
                .attestationHash(sha256(canonical(safe)))
                .diagnostics(diagnostics)
                .build();
        attestations.add(summary);
        return summary;
    }

    public List<ComplianceDtos.ComplianceAttestationSummary> attestations() {
        return new ArrayList<>(attestations);
    }

    private List<String> diagnostics(ComplianceDtos.ComplianceAttestationRequest request) {
        List<String> diagnostics = new ArrayList<>();
        if (blank(request.getOrganization())) diagnostics.add("Organization or individual name is required.");
        if (blank(request.getPurpose())) diagnostics.add("Purpose of use is required.");
        if (blank(request.getUseType())) diagnostics.add("Use type is required, such as EVALUATION, DEPLOYMENT, DERIVATIVE, INTEGRATION, or LLM_RECREATION.");
        if (blank(request.getPublicDisclosureUrl())) diagnostics.add("Public disclosure URL is required.");
        if (blank(request.getSourceCodeUrl())) diagnostics.add("Published source code URL is required.");
        if (blank(request.getOperationalConfigUrl())) diagnostics.add("Published redacted operational configuration URL is required.");
        if (!request.isAcknowledgesAgpl()) diagnostics.add("AGPL-3.0-or-later acknowledgement is required.");
        if (!request.isAcknowledgesPublicUseAndAiPolicy()) diagnostics.add("Public use and AI recreation policy acknowledgement is required.");
        if (!request.isSourcePublished()) diagnostics.add("Source publication must be confirmed.");
        if (!request.isOperationalConfigPublished()) diagnostics.add("Redacted operational configuration publication must be confirmed.");
        if (!request.isSecretsRedacted()) diagnostics.add("Secret redaction must be confirmed; do not publish credentials or sensitive data.");
        return diagnostics;
    }

    private String canonical(ComplianceDtos.ComplianceAttestationRequest request) {
        return String.join("|",
                lower(request.getOrganization()),
                lower(request.getContact()),
                lower(request.getProjectUrl()),
                lower(request.getSourceCodeUrl()),
                lower(request.getOperationalConfigUrl()),
                lower(request.getPublicDisclosureUrl()),
                lower(request.getPurpose()),
                lower(request.getUseType()),
                lower(request.getDeploymentType()),
                String.valueOf(request.isAcknowledgesAgpl()),
                String.valueOf(request.isAcknowledgesPublicUseAndAiPolicy()),
                String.valueOf(request.isSourcePublished()),
                String.valueOf(request.isOperationalConfigPublished()),
                String.valueOf(request.isSecretsRedacted()));
    }

    private String sha256(String value) {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            byte[] bytes = digest.digest(String.valueOf(value).getBytes(StandardCharsets.UTF_8));
            StringBuilder builder = new StringBuilder(bytes.length * 2);
            for (byte b : bytes) builder.append(String.format("%02x", b));
            return builder.toString();
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException("SHA-256 not available", e);
        }
    }

    private String trim(String value) {
        return value == null ? null : value.trim();
    }

    private String lower(String value) {
        return String.valueOf(trim(value)).toLowerCase(Locale.US);
    }

    private boolean blank(String value) {
        return value == null || value.trim().isEmpty();
    }
}
