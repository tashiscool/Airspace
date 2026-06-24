package org.tash.extensions.product.dto;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

public final class ComplianceDtos {
    private ComplianceDtos() {
    }

    @Data
    @Builder
    public static class CompliancePolicySummary {
        private String policyVersion;
        private String licenseSpdx;
        private String legalLicense;
        private String collectorMode;
        private boolean hiddenTelemetryEnabled;
        private boolean cloneTimeCollectionEnabled;
        private boolean sourceDisclosureRequired;
        private boolean operationalConfigDisclosureRequired;
        private boolean secretRedactionRequired;
        @Builder.Default
        private List<String> obligations = new ArrayList<>();
        @Builder.Default
        private List<String> prohibitedCollection = new ArrayList<>();
        @Builder.Default
        private List<String> documents = new ArrayList<>();
        private String legalNote;
    }

    @Data
    @Builder
    public static class ComplianceManifest {
        private String projectName;
        private String projectVersion;
        private String policyVersion;
        private String repositoryUrl;
        private String licenseSpdx;
        private ZonedDateTime generatedAt;
        private boolean hiddenTelemetryEnabled;
        private boolean cloneTimeCollectionEnabled;
        private boolean sourceDisclosureRequired;
        private boolean operationalConfigDisclosureRequired;
        private boolean secretRedactionRequired;
        private boolean collectorUrlConfigured;
        private String collectorUrl;
        private String attestationEndpoint;
        private String publicRegistryUrl;
        @Builder.Default
        private List<String> requiredDisclosureFields = new ArrayList<>();
        @Builder.Default
        private List<String> nonSecretOperationalConfigFields = new ArrayList<>();
        @Builder.Default
        private List<String> redactionRules = new ArrayList<>();
        @Builder.Default
        private List<String> diagnostics = new ArrayList<>();
    }

    @Data
    public static class ComplianceAttestationRequest {
        private String organization;
        private String contact;
        private String projectUrl;
        private String sourceCodeUrl;
        private String operationalConfigUrl;
        private String publicDisclosureUrl;
        private String purpose;
        private String useType;
        private String deploymentType;
        private String actor;
        private boolean acknowledgesAgpl;
        private boolean acknowledgesPublicUseAndAiPolicy;
        private boolean sourcePublished;
        private boolean operationalConfigPublished;
        private boolean secretsRedacted;
        private String notes;
    }

    @Data
    @Builder
    public static class ComplianceAttestationSummary {
        private String id;
        private boolean accepted;
        private String status;
        private String policyVersion;
        private ZonedDateTime receivedAt;
        private String organization;
        private String contact;
        private String projectUrl;
        private String sourceCodeUrl;
        private String operationalConfigUrl;
        private String publicDisclosureUrl;
        private String purpose;
        private String useType;
        private String deploymentType;
        private String actor;
        private boolean acknowledgesAgpl;
        private boolean acknowledgesPublicUseAndAiPolicy;
        private boolean sourcePublished;
        private boolean operationalConfigPublished;
        private boolean secretsRedacted;
        private String attestationHash;
        @Builder.Default
        private List<String> diagnostics = new ArrayList<>();
    }
}
