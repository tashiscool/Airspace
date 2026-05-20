package org.tash.extensions.messaging.transaction;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class ServiceTableCommand {
    private boolean accepted;
    private String service;
    private String domain;
    private String operation;
    private String tableName;
    private List<String> arguments;
    private List<String> warnings;
    private List<String> errors;

    public static ServiceTableCommand parse(String text) {
        ServiceRequestCommand command = ServiceRequestCommand.parseWithService(text, "TBL");
        return ServiceTableCommand.builder()
                .accepted(command.isAccepted())
                .service(command.getService())
                .domain(command.getDomain())
                .operation(command.getOperation())
                .tableName(command.getOperation())
                .arguments(command.getArguments())
                .warnings(command.getWarnings())
                .errors(command.getErrors())
                .build();
    }
}
