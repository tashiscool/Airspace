package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.PathParam;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.simulation.HistoricalReplayCalibrationReport;
import org.tash.extensions.simulation.HistoricalReplayDay;
import org.tash.extensions.simulation.HistoricalReplayLoadRequest;
import org.tash.extensions.simulation.HistoricalReplayLoadResult;
import org.tash.extensions.simulation.NationalDemandCapacityConfig;
import org.tash.extensions.simulation.NationalDemandCapacityReport;
import org.tash.extensions.simulation.OperationalSimulationService;
import org.tash.extensions.simulation.SafetyDossierExport;
import org.tash.extensions.simulation.ScenarioBundle;
import org.tash.extensions.simulation.ScenarioValidationResult;
import org.tash.extensions.simulation.SimulationAgentReport;
import org.tash.extensions.simulation.SimulationAgentRequest;
import org.tash.extensions.simulation.SimulationCampaignReport;
import org.tash.extensions.simulation.SimulationCampaignRequest;
import org.tash.extensions.simulation.SimulationReplayBundle;
import org.tash.extensions.simulation.SimulationRunRequest;
import org.tash.extensions.simulation.SimulationRunResult;
import org.tash.extensions.simulation.SimulationScenario;
import org.tash.extensions.simulation.SimulationStepResult;
import org.tash.extensions.simulation.SimulationWorldState;
import org.tash.extensions.simulation.TrafficReplayBundle;
import org.tash.extensions.simulation.TrafficReplayValidationResult;
import org.tash.extensions.visualization.AirspaceFeatureCollection;

import java.util.List;

@Path("/api/simulations")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class SimulationResource {
    @Inject
    OperationalSimulationService simulationService;

    @GET
    @Path("/scenarios")
    public List<SimulationScenario> scenarios() {
        return simulationService.scenarios();
    }

    @POST
    @Path("/scenarios/validate")
    public ScenarioValidationResult validateScenario(ScenarioBundle bundle) {
        return simulationService.validateScenario(bundle);
    }

    @POST
    @Path("/scenarios/import")
    public ScenarioBundle importScenario(ScenarioBundle bundle) {
        return simulationService.importScenario(bundle);
    }

    @POST
    @Path("/traffic-replay/validate")
    public TrafficReplayValidationResult validateTrafficReplay(TrafficReplayBundle bundle) {
        return simulationService.validateTrafficReplay(bundle);
    }

    @GET
    @Path("/historical-replay/days")
    public List<HistoricalReplayDay> historicalReplayDays() {
        return simulationService.historicalReplayDays();
    }

    @GET
    @Path("/historical-replay/days/{id}")
    public HistoricalReplayDay historicalReplayDay(@PathParam("id") String id) {
        return simulationService.historicalReplayDay(id);
    }

    @POST
    @Path("/historical-replay/load")
    public HistoricalReplayLoadResult loadHistoricalReplay(HistoricalReplayLoadRequest request) {
        return simulationService.loadHistoricalReplay(request);
    }

    @POST
    @Path("/historical-replay/calibrate")
    public HistoricalReplayCalibrationReport historicalReplayCalibrationReport(HistoricalReplayLoadRequest request) {
        return simulationService.historicalReplayCalibrationReport(request);
    }

    @POST
    @Path("/national-demand/preview")
    public NationalDemandCapacityReport previewNationalDemandCapacity(NationalDemandCapacityConfig config) {
        return simulationService.previewNationalDemandCapacity(config);
    }

    @GET
    @Path("/scenarios/{id}/bundle")
    public ScenarioBundle scenarioBundle(@PathParam("id") String id) {
        return simulationService.scenarioBundle(id);
    }

    @POST
    @Path("/run")
    public SimulationRunResult run(SimulationRunRequest request) {
        return simulationService.run(request);
    }

    @POST
    @Path("/campaign")
    public SimulationCampaignReport campaign(SimulationCampaignRequest request) {
        return simulationService.campaign(request);
    }

    @GET
    @Path("/runs/{id}")
    public SimulationRunResult runById(@PathParam("id") String id) {
        return simulationService.runById(id);
    }

    @GET
    @Path("/runs/{id}/timeline")
    public List<SimulationStepResult> timeline(@PathParam("id") String id) {
        return simulationService.timeline(id);
    }

    @GET
    @Path("/runs/{id}/world-state")
    public SimulationWorldState worldState(@PathParam("id") String id) {
        return simulationService.worldState(id);
    }

    @GET
    @Path("/runs/{id}/replay")
    public SimulationReplayBundle replay(@PathParam("id") String id) {
        return simulationService.replay(id);
    }

    @GET
    @Path("/runs/{id}/features")
    public AirspaceFeatureCollection features(@PathParam("id") String id) {
        return simulationService.features(id);
    }

    @GET
    @Path("/campaigns/{id}/report")
    public SimulationCampaignReport campaignById(@PathParam("id") String id) {
        return simulationService.campaignById(id);
    }

    @GET
    @Path("/campaigns/{id}/dossier")
    public SafetyDossierExport campaignDossier(@PathParam("id") String id) {
        return simulationService.campaignDossier(id);
    }

    @POST
    @Path("/agents/generate-scenarios")
    public SimulationAgentReport generateScenarios(SimulationAgentRequest request) {
        return simulationService.generateScenarios(request);
    }

    @POST
    @Path("/agents/red-team")
    public SimulationAgentReport redTeam(SimulationAgentRequest request) {
        return simulationService.redTeam(request);
    }
}
