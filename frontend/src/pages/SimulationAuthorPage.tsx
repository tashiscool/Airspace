import { useEffect, useMemo, useState } from 'react';
import { useMutation, useQuery } from '@tanstack/react-query';
import { FileCheck2, FlaskConical, ShieldAlert, Sparkles } from 'lucide-react';
import { api } from '../api/client';
import { ErrorNotice, MutationNotice, QueryNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import type { ScenarioBundle, SimulationAgentReport, SimulationScenario } from '../types';

export function SimulationAuthorPage() {
  const scenarios = useQuery({ queryKey: ['simulation', 'scenarios'], queryFn: api.simulationScenarios });
  const [selectedScenarioId, setSelectedScenarioId] = useState('low-vis-rvr-smgcs');
  const bundle = useQuery({
    queryKey: ['simulation', 'scenario-bundle', selectedScenarioId],
    queryFn: () => api.simulationScenarioBundle(selectedScenarioId),
    enabled: !!selectedScenarioId
  });
  const [bundleText, setBundleText] = useState('');
  const [agentReport, setAgentReport] = useState<SimulationAgentReport | undefined>();
  const parsedBundle = useMemo(() => parseBundle(bundleText), [bundleText]);
  const validate = useMutation({
    mutationFn: () => {
      if (!parsedBundle.bundle) throw new Error(parsedBundle.error ?? 'Scenario bundle JSON is invalid.');
      return api.validateSimulationScenario(parsedBundle.bundle);
    }
  });
  const importScenario = useMutation({
    mutationFn: () => {
      if (!parsedBundle.bundle) throw new Error(parsedBundle.error ?? 'Scenario bundle JSON is invalid.');
      return api.importSimulationScenario(parsedBundle.bundle);
    }
  });
  const generate = useMutation({
    mutationFn: () => api.generateSimulationScenarios({
      scenarioType: 'LOW_VISIBILITY_PROCEDURE_AMBIGUITY',
      count: 2,
      focusAreas: ['RVR', 'SMGCS', 'weather-route-impact', 'source-refs']
    }),
    onSuccess: (report) => {
      setAgentReport(report);
      const draft = report.generatedScenarioDrafts[0];
      if (draft) setBundleText(JSON.stringify(draft, null, 2));
    }
  });
  const redTeam = useMutation({
    mutationFn: () => api.redTeamSimulation({ count: 1, focusAreas: ['false-clear', 'false-block', 'source-refs'] }),
    onSuccess: setAgentReport
  });

  useEffect(() => {
    if (bundle.data) setBundleText(JSON.stringify(bundle.data, null, 2));
  }, [bundle.data]);

  return (
    <section className="simulation-workbench">
      <aside className="decision-queue">
        <header>Scenario Authoring</header>
        {(scenarios.data ?? []).map((scenario) => (
          <button
            key={scenario.id}
            className={scenario.id === selectedScenarioId ? 'queue-item active' : 'queue-item'}
            onClick={() => setSelectedScenarioId(scenario.id)}
          >
            <span>{scenario.name}</span>
            <StatusBadge value={scenario.expectedFinalAction ?? 'SIM'} />
            <small>{scenario.capabilityStory}</small>
          </button>
        ))}
      </aside>
      <main className="decision-detail">
        <div className="page-header">
          <div>
            <h2><FlaskConical size={18} /> Simulation Scenario Authoring</h2>
            <p>Create, validate, import, and red-team local aerospace simulation bundles. Drafts require human review before they become regression scenarios.</p>
          </div>
          <StatusBadge value={validate.data?.accepted ? 'ACCEPTED' : parsedBundle.bundle ? 'DRAFT' : 'INVALID'} />
        </div>
        <div className="notice-stack">
          <QueryNotice query={scenarios} label="Simulation scenarios" />
          <QueryNotice query={bundle} label="Scenario bundle" />
          <ErrorNotice error={validate.error ?? importScenario.error ?? generate.error ?? redTeam.error} title="Scenario authoring failed" />
          <MutationNotice mutation={validate} label="Validate scenario" />
          <MutationNotice mutation={importScenario} label="Import scenario" />
          <MutationNotice mutation={generate} label="Generate draft" />
          <MutationNotice mutation={redTeam} label="Red-team scenario" />
        </div>
        <section className="panel">
          <div className="panel-heading">
            <h3><FileCheck2 size={15} /> Scenario Bundle JSON</h3>
            <span>{selectedScenarioId}</span>
          </div>
          <div className="toolbar compact">
            <button type="button" onClick={() => validate.mutate()} disabled={!parsedBundle.bundle || validate.isPending}>Validate</button>
            <button type="button" onClick={() => importScenario.mutate()} disabled={!parsedBundle.bundle || importScenario.isPending}>Import</button>
            <button type="button" className="secondary" onClick={() => generate.mutate()} disabled={generate.isPending}><Sparkles size={14} /> Generate Draft</button>
            <button type="button" className="secondary" onClick={() => redTeam.mutate()} disabled={redTeam.isPending}><ShieldAlert size={14} /> Red-Team</button>
          </div>
          <textarea
            className="raw-panel scenario-author-textarea"
            value={bundleText}
            onChange={(event) => setBundleText(event.target.value)}
            spellCheck={false}
          />
          {parsedBundle.error && <p className="error">{parsedBundle.error}</p>}
        </section>
        <section className="simulation-grid">
          <ValidationPanel accepted={validate.data?.accepted} warnings={validate.data?.warnings ?? []} errors={validate.data?.errors ?? []} />
          <BundleSummary scenario={parsedBundle.bundle?.scenario ?? bundle.data?.scenario} />
          <AgentPanel report={agentReport} />
        </section>
      </main>
    </section>
  );
}

function parseBundle(value: string): { bundle?: ScenarioBundle; error?: string } {
  if (!value.trim()) return {};
  try {
    return { bundle: JSON.parse(value) as ScenarioBundle };
  } catch (error) {
    return { error: error instanceof Error ? error.message : 'Invalid JSON.' };
  }
}

function ValidationPanel({ accepted, warnings, errors }: { accepted?: boolean; warnings: string[]; errors: string[] }) {
  return (
    <section className="panel">
      <div className="panel-heading"><h3>Validation Gate</h3><StatusBadge value={accepted ? 'PASS' : errors.length ? 'FAIL' : 'NOT RUN'} /></div>
      <ul className="trace-list">
        {errors.map((item) => <li key={item} className="error">{item}</li>)}
        {warnings.map((item) => <li key={item}>{item}</li>)}
        {!errors.length && !warnings.length && <li className="muted">No validation diagnostics yet.</li>}
      </ul>
    </section>
  );
}

function BundleSummary({ scenario }: { scenario?: SimulationScenario }) {
  return (
    <section className="panel">
      <div className="panel-heading"><h3>Scenario Summary</h3><span>{scenario?.events.length ?? 0} event(s)</span></div>
      {scenario ? (
        <div className="timeline-list">
          <p><strong>{scenario.name}</strong></p>
          <p>{scenario.narrative}</p>
          <div className="source-ref-grid">
            <span>Route points {scenario.route.length}</span>
            <span>Expected {scenario.expectedFinalAction ?? 'review'}</span>
            <span>{scenario.capabilityStory ?? 'Simulation'}</span>
          </div>
        </div>
      ) : <p className="empty-state">No scenario bundle loaded.</p>}
    </section>
  );
}

function AgentPanel({ report }: { report?: SimulationAgentReport }) {
  return (
    <section className="panel wide">
      <div className="panel-heading"><h3>Autonomous Workloads</h3><span>advisory only</span></div>
      {report ? (
        <div className="timeline-list">
          <p><strong>{report.agentType}</strong></p>
          <div className="source-ref-grid">
            <span>{report.generatedScenarioDrafts.length} draft(s)</span>
            <span>{report.findings.length} finding(s)</span>
            <span>{report.policyGuards.join(', ')}</span>
          </div>
          {report.findings.map((finding) => <article className="event supplement" key={finding}>{finding}</article>)}
        </div>
      ) : <p className="empty-state">Generate drafts or run the red-team workload to create reviewable agent output.</p>}
    </section>
  );
}
