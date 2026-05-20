package org.tash.extensions.messaging;

import org.tash.extensions.messaging.transaction.UsnsTransaction;
import org.tash.extensions.messaging.transaction.UsnsTransactionType;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class CarfMessageFamilyParser {
    public CarfMessageFamilyParseResult parse(UsnsTransaction transaction) {
        List<String> warnings = new ArrayList<>();
        List<String> errors = new ArrayList<>();
        Map<String, String> fields = new LinkedHashMap<>();
        if (transaction == null) {
            errors.add("No transaction supplied");
            return result(null, CarfMessageFamilyStatus.REJECTED, "", fields, "none", warnings, errors);
        }
        String raw = transaction.getRawText() == null ? "" : transaction.getRawText();
        String normalized = raw.toUpperCase(Locale.US).replaceAll("\\s+", " ").trim();
        UsnsTransactionType type = transaction.getType();
        fields.put("family", type.name());
        fields.put("preview", normalized.length() > 120 ? normalized.substring(0, 120) : normalized);
        Matcher notam = Pattern.compile("\\bNOTAM([NRCJ])\\b").matcher(normalized);
        if (notam.find()) {
            fields.put("notamAction", notam.group(1));
        }
        Matcher snowtam = Pattern.compile("\\bSNOWTAM\\s+([A-Z0-9/.-]+)?").matcher(normalized);
        if (snowtam.find()) {
            fields.put("snowtamId", value(snowtam.group(1)));
            fields.put("lifecycle", "replace-prior-snowtam-for-same-account-source-location");
        }
        Matcher birdtam = Pattern.compile("\\bBIRDTAM\\s+([A-Z0-9/.-]+)?").matcher(normalized);
        if (birdtam.find()) {
            fields.put("birdtamId", value(birdtam.group(1)));
            fields.put("lifecycle", "validate-duplicate-and-advisory-filter");
        }
        Matcher ashtam = Pattern.compile("\\bASHTAM\\s+([A-Z0-9/.-]+)?").matcher(normalized);
        if (ashtam.find()) {
            fields.put("ashtamId", value(ashtam.group(1)));
            fields.put("lifecycle", "admin-advisory-only");
        }
        Matcher genot = Pattern.compile("\\bGENOT\\s+([A-Z0-9]+)?").matcher(normalized);
        if (genot.find()) {
            fields.put("genotSeries", value(genot.group(1)));
            fields.put("lifecycle", "admin-message");
        }
        Matcher fdc = Pattern.compile("\\bFDC\\s+(\\d+/\\d+)").matcher(normalized);
        if (fdc.find()) {
            fields.put("fdcId", fdc.group(1));
        }
        if (type == UsnsTransactionType.UNKNOWN) {
            errors.add("Unknown transaction family");
            return result(type, CarfMessageFamilyStatus.REJECTED, raw, fields, "unknown", warnings, errors);
        }
        CarfMessageFamilyStatus status = status(type);
        if (status == CarfMessageFamilyStatus.CLASSIFIED_ONLY) {
            warnings.add("Classified family retained without full domain parser: " + type);
        }
        return result(type, status, raw, fields, semantic(type), warnings, errors);
    }

    private CarfMessageFamilyParseResult result(UsnsTransactionType type,
                                                CarfMessageFamilyStatus status,
                                                String raw,
                                                Map<String, String> fields,
                                                String semantic,
                                                List<String> warnings,
                                                List<String> errors) {
        return CarfMessageFamilyParseResult.builder()
                .type(type)
                .status(status)
                .rawText(raw)
                .fields(Collections.unmodifiableMap(new LinkedHashMap<>(fields)))
                .semantic(semantic)
                .warnings(Collections.unmodifiableList(new ArrayList<>(warnings)))
                .errors(Collections.unmodifiableList(new ArrayList<>(errors)))
                .build();
    }

    private CarfMessageFamilyStatus status(UsnsTransactionType type) {
        switch (type) {
            case SNOWTAM:
            case BIRDTAM:
            case ASHTAM:
            case GENOT:
            case SERVICE_REQUEST:
            case SERVICE_TABLE:
            case FDC_ACK:
                return CarfMessageFamilyStatus.SUPPORTED;
            case UNKNOWN:
                return CarfMessageFamilyStatus.REJECTED;
            default:
                return CarfMessageFamilyStatus.CLASSIFIED_ONLY;
        }
    }

    private String semantic(UsnsTransactionType type) {
        switch (type) {
            case SNOWTAM:
                return "snowtam-replaces-prior-active-record";
            case BIRDTAM:
                return "birdtam-duplicate-and-advisory-policy";
            case ASHTAM:
                return "ashtam-admin-advisory";
            case GENOT:
                return "genot-admin-message";
            case CANADIAN_DOMESTIC:
                return "canadian-domestic-notam";
            case FDC:
                return "fdc-non-laser-or-laser";
            case ICAO_NOTAMN:
            case ICAO_NOTAMR:
            case ICAO_NOTAMC:
                return "icao-notam";
            default:
                return type == null ? "" : type.name().toLowerCase(Locale.US);
        }
    }

    private String value(String value) {
        return value == null ? "" : value;
    }
}
