import { expect, test, type Route } from '@playwright/test';

const missionId = 'mission-e2e';
const reservationId = 'reservation-e2e';
const decisionId = 'decision-e2e';
const messageId = 'message-e2e';
let taskStatus = 'OPEN';

test.beforeEach(async ({ page }) => {
  taskStatus = 'OPEN';
  await page.route('**/*', async (route) => {
    const path = new URL(route.request().url()).pathname;
    if (path.startsWith('/api/')) {
      await respond(route);
      return;
    }
    await route.continue();
  });
});

test('agentic operations smoke covers run, trace, task acknowledgment, coordination, and pilot brief', async ({ page }) => {
  await page.goto('http://127.0.0.1:5173/login');
  await expect(page.getByText('Airspace Operations')).toBeVisible();
  await page.getByText('Sign in').click();
  await expect(page.locator('.board-title', { hasText: 'Mission Explorer' })).toBeVisible();

  await page.getByRole('button', { name: /Agentic Ops/i }).click();
  await expect(page.getByRole('button', { name: /Run all agents/i })).toBeVisible();

  await page.getByRole('button', { name: /Run all agents/i }).click();
  await expect(page.getByRole('heading', { name: 'Findings' })).toBeVisible();
  await expect(page.getByText(/Agentic operations review/i)).toBeVisible();
  await expect(page.getByText(/HITL push approval/i)).toBeVisible();
  await expect(page.getByRole('heading', { name: 'Risk Checklist' })).toBeVisible();

  await page.getByPlaceholder(/Ask against decision trace/i).fill('Why this reroute?');
  await page.getByRole('button', { name: 'Ask trace' }).click();
  await expect(page.getByText('Trace Answer')).toBeVisible();
  await expect(page.getByText(/rule-route-blockage/i)).toBeVisible();

  await page.getByRole('button', { name: /Acknowledge/i }).first().click();
  await expect(page.getByText(/acknowledged/i).first()).toBeVisible();

  await page.getByRole('button', { name: /Draft coordination/i }).click();
  await expect(page.getByText(/Coordinate reroute/i).first()).toBeVisible();

  await page.goto(`/missions/${missionId}/brief`);
  await expect(page.getByText(/Pilot Brief/i)).toBeVisible();
  await expect(page.getByText('Pilot Brief REROUTE SIGMET:E2E')).toBeVisible();
});

async function respond(route: Route) {
  const url = new URL(route.request().url());
  const path = url.pathname;
  const method = route.request().method();
  if (path === '/api/auth/login') return json(route, login());
  if (path === '/api/auth/me') return json(route, login().user);
  if (path === '/api/missions') return json(route, [mission()]);
  if (path === `/api/missions/${missionId}`) return json(route, missionDetail());
  if (path === `/api/missions/${missionId}/weather-verdict`) return json(route, verdict());
  if (path === `/api/missions/${missionId}/weather-changes`) return json(route, verdict().sources);
  if (path === `/api/missions/${missionId}/route-impact`) return json(route, routeImpact());
  if (path === `/api/missions/${missionId}/pilot-brief`) return json(route, pilotBrief());
  if (path === `/api/missions/${missionId}/coordinate-weather`) return json(route, coordinationDraft());
  if (path === '/api/weather/affected-missions') return json(route, [affectedMission()]);
  if (path === '/api/messages') return json(route, [message()]);
  if (path === `/api/messages/${messageId}`) return json(route, message());
  if (path === '/api/feed/artifacts') return json(route, []);
  if (path === '/api/decisions/latest' || path === `/api/decisions/${decisionId}`) return json(route, decision());
  if (path === '/api/history') return json(route, []);
  if (path === '/api/search') return json(route, []);
  if (path === '/api/reference/points') return json(route, []);
  if (path === '/api/config') return json(route, { agenticStore: 'IN_MEMORY', agenticStoreDurable: false });
  if (path === '/api/metrics') return json(route, { 'agentic.runs': 1, 'agentic.tasks': 1 });
  if (path === '/api/agents/status') return json(route, { mode: 'IN_MEMORY', durable: false, runCount: 1, taskCount: 1 });
  if (path === '/api/agents/metrics') return json(route, { 'agentic.runs.accepted': 1, 'agentic.policyViolations': 0 });
  if (path === '/api/agents/risk-assessments') return json(route, riskAssessments());
  if (path === '/api/agents/evaluate/stability') return json(route, stabilityResult());
  if (path === '/api/agents/mcp/servers') return json(route, mcpServers());
  if (path === '/api/agents/mcp/servers/airspace-first-party/tools') return json(route, mcpTools());
  if (path === '/api/agents/mcp/receipts') return json(route, [mcpReceipt()]);
  if (path === '/api/agents/jobs') {
    if (method === 'POST') return json(route, agentJob());
    return json(route, [agentJob()]);
  }
  if (path === '/api/agents/runs') return json(route, [agentResult('ALL')]);
  if (path === '/api/agents/tasks') return json(route, [agentTask(taskStatus)]);
  if (path === '/api/agents/tasks/task-e2e/transition' && method === 'POST') {
    taskStatus = 'ACKNOWLEDGED';
    return json(route, agentTask(taskStatus));
  }
  if (path === '/api/agents/replay-audit') return json(route, agentResult('REPLAY_AUDIT', true));
  if (path === '/api/agents/coordination-draft') return json(route, agentResult('COORDINATION_DRAFT'));
  if (path.startsWith('/api/agents/')) return json(route, agentResult(path.includes('mission-risk') ? 'MISSION_RISK' : 'ALL'));
  return json(route, {});
}

function json(route: Route, body: unknown) {
  return route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify(body)
  });
}

function login() {
  return { accepted: true, token: 'e2e-token', user: { id: 'planner', username: 'planner', displayName: 'Planner', roles: ['planner'] }, diagnostics: [] };
}

function mission() {
  return { id: missionId, missionNumber: 'E2E-NXGEN', title: 'Agentic E2E Mission', status: 'ACTIVE', reservationCount: 1 };
}

function missionDetail() {
  return { mission: mission(), reservations: [{ id: reservationId, missionId, state: 'DRAFT', rawText: 'A. E2E-NXGEN', conflictCount: 1, diagnostics: [] }], messages: [message()], history: [] };
}

function message() {
  return { id: messageId, missionId, reservationId, family: 'SIGMET', direction: 'INBOUND', status: 'RECEIVED', subject: 'E2E SIGMET', rawText: 'SIGMET E2E VALID 2000/2400 FROM 3000N15000W TO 3100N14900W EMBD TS' };
}

function verdict() {
  return {
    missionId,
    action: 'REROUTE',
    priority: 'HIGH',
    confidence: 0.88,
    sourceCount: 1,
    stale: false,
    summary: 'Convective weather intersects route.',
    recommendedAction: 'Coordinate reroute',
    diagnostics: [],
    sources: [{ id: 'E2E', family: 'SIGMET', label: 'Convective SIGMET', severity: 'SEVERE', rationale: 'Route overlap', stale: false }]
  };
}

function routeImpact() {
  return {
    missionId,
    reservationId,
    action: 'REROUTE',
    recommendedAction: 'Coordinate reroute',
    confidence: 0.87,
    rationale: 'Weather intersects segment 2.',
    impactedSegmentCount: 1,
    blockingConstraintCount: 1,
    impactedSegments: ['segment-2'],
    sourceRefs: ['SIGMET:E2E'],
    avoidanceCandidates: ['north deviation'],
    candidateComparisons: [{ id: 'cand-1', label: 'North deviation', confidence: 0.82, sourceRefs: ['SIGMET:E2E'], avoidedConstraints: [{ id: 'SIGMET:E2E', family: 'WEATHER', label: 'Convective SIGMET' }], residualConstraints: [], cost: { additionalDistanceNm: 42, additionalMinutes: 8, additionalFuelLb: 900, additionalCostUsd: 3200 } }],
    whyRerouteTrace: [{ ruleId: 'rule-route-blockage', message: 'Avoid severe convection.', sourceRef: 'SIGMET:E2E' }],
    diagnostics: []
  };
}

function decision() {
  return { id: decisionId, action: 'REROUTE', recommendedAction: 'Coordinate reroute', confidence: 0.87, rationale: 'Weather intersects route.', routeImpact: routeImpact() };
}

function pilotBrief() {
  return { missionId, missionNumber: 'E2E-NXGEN', generatedAt: '2026-05-22T00:00:00Z', verdict: verdict(), routeImpact: routeImpact(), coordinationDraft: coordinationDraft(), changes: verdict().sources, sourceSummaryLines: ['SIGMET:E2E severe convection'], decisionTraceSummary: 'rule-route-blockage SIGMET:E2E', printableText: 'Pilot Brief REROUTE SIGMET:E2E' };
}

function coordinationDraft() {
  return { id: 'draft-e2e', missionId, reservationId, subject: 'Draft USNS coordination', family: 'USNS', direction: 'OUTBOUND', rawText: 'Coordinate reroute for SIGMET:E2E', recommendedAction: 'Coordinate reroute', recipients: ['WX-DESK'], sourceRefs: ['SIGMET:E2E'] };
}

function affectedMission() {
  return { missionId, missionNumber: 'E2E-NXGEN', status: 'ACTIVE', action: 'REROUTE', priority: 'HIGH', confidence: 0.88, sourceCount: 1, impactedSegmentCount: 1, blockingConstraintCount: 1, ageSeconds: 1, stale: false, guidanceLatencySeconds: 1, guidanceTargetMet: true, sourceRefs: ['SIGMET:E2E'] };
}

function agentTask(status: string) {
  return { id: 'task-e2e', title: 'Review and approve coordination draft', status, priority: 'HIGH', assignedRole: 'planner', route: `/missions/${missionId}`, rationale: `${status.toLowerCase()} task from E2E`, humanReviewMode: 'PUSH_APPROVAL', humanReviewReason: 'Draft-only output requires approval.', citations: [{ sourceFamily: 'SIGMET', sourceId: 'E2E' }] };
}

function agentResult(agentType: string, trace = false) {
  return {
    id: `agent-${agentType}`,
    agentType,
    missionId,
    reservationId,
    decisionId,
    summary: agentType === 'COORDINATION_DRAFT' ? 'Draft USNS coordination message: Draft USNS coordination' : 'Agentic operations review produced 1 finding(s), 1 recommendation(s), and 1 task(s).',
    confidence: 0.84,
    accepted: true,
    generatedAt: '2026-05-22T00:00:00Z',
    findings: [{ id: 'finding-e2e', category: 'MISSION_RISK', severity: 'HIGH', message: 'Weather impact requires review.', citations: [{ sourceFamily: 'SIGMET', sourceId: 'E2E' }] }],
    recommendations: [{ id: 'rec-e2e', action: 'REROUTE', summary: 'Coordinate reroute', rationale: 'Avoid severe weather.', confidence: 0.84, humanApprovalRequired: true, humanReviewMode: 'PUSH_APPROVAL', humanReviewReason: 'Draft-only output requires approval.', citations: [{ sourceFamily: 'SIGMET', sourceId: 'E2E' }] }],
    tasks: [agentTask('OPEN')],
    assessments: [{ id: 'assessment-e2e', schemaVersion: 'agent-assessment-v1', claim: 'Weather impact requires review.', verdict: 'HIGH', confidence: 0.84, evidence: ['SIGMET:E2E'], counterEvidence: [], uncertainty: 'Bounded by cited local artifacts.', requiredHumanAction: 'Approve, edit, or reject draft.', humanReviewMode: 'PUSH_APPROVAL', citations: [{ sourceFamily: 'SIGMET', sourceId: 'E2E' }] }],
    citations: [{ sourceFamily: 'SIGMET', sourceId: 'E2E' }],
    toolCalls: [{ id: 'tool-e2e', toolName: 'airspace.mission.weather_verdict', serverId: 'airspace-first-party', sideEffectLevel: 'READ_ONLY', status: 'ACCEPTED', evidenceReceiptId: 'receipt-e2e' }],
    deltas: [{ id: 'delta-e2e', changeType: 'ACTION_CHANGED', sourceFamily: 'DECISION', sourceId: decisionId, previousValue: 'CAUTION', currentValue: 'REROUTE', severity: 'HIGH', citations: [{ sourceFamily: 'SIGMET', sourceId: 'E2E' }] }],
    operatingLoop: [{ stage: 'OBSERVE', status: 'COMPLETE', summary: 'Observed weather and mission route.', citations: [{ sourceFamily: 'SIGMET', sourceId: 'E2E' }] }],
    traceAnswer: trace ? { question: 'Why this reroute?', answer: 'Reroute because SIGMET:E2E intersects the route.', ruleIds: ['rule-route-blockage'], sourceRefs: ['SIGMET:E2E'], citations: [{ sourceFamily: 'SIGMET', sourceId: 'E2E' }] } : undefined,
    auditEnvelope: { inputHash: 'input-e2e', outputHash: 'output-e2e', policyVersion: 'agent-policy-v1' },
    evaluation: { accepted: true, citationCoverage: 1, policyViolationCount: 0, citedClaimCount: 3, uncitedClaimCount: 0 },
    reasoningEnvelope: { reasoningMode: 'DETERMINISTIC_DRAFT_ONLY', modelId: 'deterministic-draft-only', promptVersion: 'agentic-nextgen-v1', draftHash: 'draft-e2e', availableTools: ['airspace.mission.weather_verdict [READ_ONLY]'], blockedTools: ['external-mcp.configured_tool [REQUIRES_APPROVAL]'], toolPolicySummary: 'MCP tools provide cited evidence and drafts only.', toolReceiptIds: ['receipt-e2e'] },
    diagnostics: []
  };
}

function mcpServers() {
  return [
    { id: 'airspace-first-party', name: 'Airspace first-party tools', enabled: true, setupRequired: false, description: 'Local Airspace evidence and draft tools.' },
    { id: 'external-mcp', name: 'External MCP servers', enabled: false, setupRequired: true, description: 'Setup required.' }
  ];
}

function mcpTools() {
  return [
    { id: 'airspace.mission.weather_verdict', serverId: 'airspace-first-party', sideEffectLevel: 'READ_ONLY', description: 'Mission verdict.', requiredArguments: ['missionId'], riskProfile: riskProfile('REVIEW_ONLY') },
    { id: 'airspace.coordination.draft', serverId: 'airspace-first-party', sideEffectLevel: 'DRAFT_ONLY', description: 'Draft coordination.', requiredArguments: ['missionId'], riskProfile: riskProfile('PUSH_APPROVAL') }
  ];
}

function riskProfile(requiredHumanReviewMode: string) {
  return {
    worstCaseBlastRadius: requiredHumanReviewMode === 'PUSH_APPROVAL' ? 'A misleading draft could confuse an operator if approved without review.' : 'Incorrect local evidence summary.',
    dataEgress: 'None by default.',
    rollbackPath: 'Disable MCP catalog.',
    toolProvenance: 'Airspace first-party Java tool.',
    modelProvenance: 'Deterministic no-model invocation.',
    requiredHumanReviewMode
  };
}

function riskAssessments() {
  return mcpTools().map((tool) => ({
    id: `risk-${tool.id}`,
    subjectType: 'MCP_TOOL',
    subjectId: tool.id,
    summary: `${tool.id} risk`,
    riskProfile: tool.riskProfile,
    diagnostics: []
  }));
}

function stabilityResult() {
  return {
    id: 'stability-e2e',
    accepted: true,
    iterations: 3,
    runIds: ['run-a', 'run-b', 'run-c'],
    metrics: [
      { id: 'accepted_agreement', name: 'Accepted/verdict agreement', value: 1, threshold: 1, accepted: true },
      { id: 'cited_source_jaccard', name: 'Cited-source Jaccard', value: 1, threshold: 0.95, accepted: true }
    ],
    diagnostics: []
  };
}

function mcpReceipt() {
  return { id: 'receipt-e2e', serverId: 'airspace-first-party', toolId: 'airspace.mission.weather_verdict', status: 'ACCEPTED', policyDecision: 'ALLOWED_READ_ONLY', redactionStatus: 'CLEAN', inputHash: 'input-receipt', outputHash: 'output-receipt', durationMillis: 4, sourceRefs: [{ sourceFamily: 'MISSION', sourceId: missionId }] };
}

function agentJob() {
  return { id: 'job-e2e', status: 'SUCCEEDED', request: { agentRunRequest: { agentType: 'MISSION_RISK', missionId } }, runResult: agentResult('MISSION_RISK'), toolCalls: [agentResult('MISSION_RISK').toolCalls[0]], receipts: [mcpReceipt()], diagnostics: [] };
}
