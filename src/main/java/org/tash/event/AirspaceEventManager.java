package org.tash.event;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
     * Manager for airspace events
     */
    public class AirspaceEventManager {
        private Map<String, List<AirspaceEventListener>> listeners = new HashMap<>();
        
        /**
         * Register a listener for a specific event type
         */
        public void addEventListener(String eventType, AirspaceEventListener listener) {
            if (!listeners.containsKey(eventType)) {
                listeners.put(eventType, new ArrayList<>());
            }
            listeners.get(eventType).add(listener);
        }
        
        /**
         * Remove a listener
         */
        public void removeEventListener(String eventType, AirspaceEventListener listener) {
            if (listeners.containsKey(eventType)) {
                listeners.get(eventType).remove(listener);
            }
        }
        
        /**
         * Fire an event
         */
        public void fireEvent(AirspaceEvent event) {
            if (listeners.containsKey(event.getEventType())) {
                for (AirspaceEventListener listener : listeners.get(event.getEventType())) {
                    listener.onEvent(event);
                }
            }
            
            // Also notify "ALL" listeners
            if (listeners.containsKey("ALL")) {
                for (AirspaceEventListener listener : listeners.get("ALL")) {
                    listener.onEvent(event);
                }
            }
        }

    public void addEventListener(String eventType, Consumer<AirspaceEvent> listener) {
        addEventListener(eventType, new AirspaceEventListener() {
            @Override
            public void onEvent(AirspaceEvent event) {
                listener.accept(event);
            }
        });
    }
}