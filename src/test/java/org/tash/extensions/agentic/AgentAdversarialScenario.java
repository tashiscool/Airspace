package org.tash.extensions.agentic;

import java.util.ArrayList;
import java.util.List;

class AgentAdversarialScenario {
    public String id;
    public String description;
    public List<String> expectedCategories = new ArrayList<>();
    public List<String> forbiddenCategories = new ArrayList<>();
    public HumanReviewMode expectedReviewMode;
    public List<MessageFixture> messages = new ArrayList<>();
    public List<FeedFixture> feeds = new ArrayList<>();

    static class MessageFixture {
        public String family;
        public String subject;
        public String rawText;
    }

    static class FeedFixture {
        public String sourceId;
        public String type;
        public String body;
    }
}
