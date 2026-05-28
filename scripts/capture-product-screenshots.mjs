import fs from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(__dirname, '..');
const require = createRequire(path.join(root, 'frontend', 'package.json'));
const { chromium } = require('@playwright/test');
const frontendUrl = process.env.AIRSPACE_FRONTEND_URL ?? 'http://127.0.0.1:5173';
const apiUrl = process.env.AIRSPACE_API_URL ?? 'http://127.0.0.1:8090';
const screenshotDir = path.join(root, 'docs', 'screenshots');
const carfText = await fs.readFile(path.join(root, 'src/test/resources/scenarios/product-replay/carf-altrv.txt'), 'utf8');
const usnsText = await fs.readFile(path.join(root, 'src/test/resources/scenarios/product-replay/usns-mixed-weather.txt'), 'utf8');

async function api(pathname, options = {}) {
  const response = await fetch(`${apiUrl}${pathname}`, {
    ...options,
    headers: {
      'Content-Type': 'application/json',
      ...(options.headers ?? {})
    }
  });
  if (!response.ok) {
    const text = await response.text();
    throw new Error(`${options.method ?? 'GET'} ${pathname} failed: ${response.status} ${text}`);
  }
  if (response.status === 204) return undefined;
  return response.json();
}

async function seedProductDemo() {
  const login = await api('/api/auth/login', {
    method: 'POST',
    body: JSON.stringify({ username: 'planner', password: 'planner' })
  });

  const mission = await api('/api/missions', {
    method: 'POST',
    body: JSON.stringify({
      missionNumber: 'CARF-WX-260521',
      title: 'NextGen Weather Route Blockage Demonstration',
      rawText: carfText,
      actor: 'planner'
    })
  });

  const reservationResult = await api(`/api/missions/${mission.id}/reservations`, {
    method: 'POST',
    body: JSON.stringify({ rawText: carfText, actor: 'planner' })
  });
  const reservationId = reservationResult.record.id;

  await api(`/api/reservations/${reservationId}/parse`, {
    method: 'POST',
    body: JSON.stringify({ actor: 'planner' })
  });
  await api(`/api/reservations/${reservationId}/deconflict`, {
    method: 'POST',
    body: JSON.stringify({ actor: 'planner' })
  }).catch(() => undefined);

  await api(`/api/reservations/${reservationId}/supplements`, {
    method: 'POST',
    body: JSON.stringify({
      kind: 'NOTAM',
      status: 'OPEN',
      title: 'NOTAM constraint review',
      text: '!DCA DCA AIRSPACE ROUTE BLOCKAGE REVIEW 2605211200-2605211800',
      actor: 'planner'
    })
  });
  await api(`/api/reservations/${reservationId}/supplements`, {
    method: 'POST',
    body: JSON.stringify({
      kind: 'APREQ',
      status: 'SUBMITTED',
      title: 'APREQ coordination pending',
      text: 'APREQ required due weather reroute and protected-volume coordination.',
      actor: 'planner'
    })
  });
  await api(`/api/reservations/${reservationId}/supplements`, {
    method: 'POST',
    body: JSON.stringify({
      kind: 'APPROVAL',
      status: 'DRAFT',
      title: 'Supervisor review',
      text: 'Awaiting supervisor conflict and weather-risk acceptance.',
      actor: 'planner'
    })
  });

  const message = await api('/api/messages/send', {
    method: 'POST',
    body: JSON.stringify({
      missionId: mission.id,
      reservationId,
      family: 'USNS',
      direction: 'INBOUND',
      subject: 'Mixed USNS weather and CARF advisory',
      rawText: usnsText,
      actor: 'planner'
    })
  });
  await api('/api/messages/send', {
    method: 'POST',
    body: JSON.stringify({
      missionId: mission.id,
      reservationId,
      family: 'PIREP',
      direction: 'INBOUND',
      subject: 'Urgent turbulence PIREP',
      rawText: 'UA /OV KJFK/TM 2000/FL240/TP B738/TB SEV/RM URGENT ROUTE IMPACT',
      actor: 'planner'
    })
  });
  await api('/api/messages/send', {
    method: 'POST',
    body: JSON.stringify({
      missionId: mission.id,
      reservationId,
      family: 'SIGMET',
      direction: 'INBOUND',
      subject: 'Convective SIGMET route blockage',
      rawText: 'SIGMET DEMO 1 VALID 200000/200400 FROM 3000N15000W TO 3100N14900W EMBD TS MOV E 25KT TOP FL450 INTSF',
      actor: 'planner'
    })
  });
  await api('/api/messages/send', {
    method: 'POST',
    body: JSON.stringify({
      missionId: mission.id,
      reservationId,
      family: 'METAR',
      direction: 'INBOUND',
      subject: 'Low ceiling METAR',
      rawText: 'METAR KJFK 200000Z 18012G22KT 1/2SM +TSRA BKN004 OVC010 18/16 A2992',
      actor: 'planner'
    })
  });

  const feed = await api('/api/feed/ingest', {
    method: 'POST',
    body: JSON.stringify({
      sourceId: 'README-DEMO-USNS',
      type: 'USNS',
      rawPayload: usnsText
    })
  });
  await api('/api/feed/ingest', {
    method: 'POST',
    body: JSON.stringify({
      sourceId: 'README-DEMO-AWC-SIGMET',
      type: 'WEATHER',
      rawPayload: 'SIGMET README 1 VALID 211200/211800 FROM 3000N15000W TO 3100N14900W TO 3050N14850W EMBD TS MOV E 25KT TOP FL450 INTSF'
    })
  });
  await api('/api/feed/ingest', {
    method: 'POST',
    body: JSON.stringify({
      sourceId: 'README-DEMO-AWC-PIREP',
      type: 'PIREP',
      rawPayload: 'UA /OV 3050N14950W/TM 1215/FL240/TP B738/TB SEV/RM URGENT WEATHER PATTERN SAMPLE'
    })
  });
  await api('/api/feed/ingest', {
    method: 'POST',
    body: JSON.stringify({
      sourceId: 'README-DEMO-AWC-METAR',
      type: 'WEATHER',
      rawPayload: 'METAR KJFK 211200Z 18012G22KT 1/2SM +TSRA BKN004 OVC010 18/16 A2992'
    })
  });

  await api('/api/reference/import/apply', {
    method: 'POST',
    body: JSON.stringify({
      actor: 'admin',
      apply: true,
      payload: [
        'JFK,NAVAID,40.6398,-73.7789,13,README-DEMO',
        'DOV,NAVAID,39.1295,-75.4660,24,README-DEMO',
        'FIXA,FIX,30.0,-150.0,0,README-DEMO',
        'FIXB,FIX,31.0,-149.0,0,README-DEMO'
      ].join('\\n')
    })
  });

  const decision = await api('/api/decisions/evaluate', {
    method: 'POST',
    body: JSON.stringify({
      rawUsnsMessages: [usnsText],
      rawCarfMessages: [carfText],
      missionId: mission.id,
      reservationId,
      route: [[30.0, -150.5, 24000], [30.5, -149.8, 25000], [31.0, -149.0, 26000]],
      decisionTime: '2026-05-21T12:00:00Z'
    })
  });
  await api(`/api/decisions/${decision.id}/replay`, { method: 'POST', body: '{}' }).catch(() => undefined);

  return { token: login.token, missionId: mission.id, reservationId, messageId: message.id, feedId: feed.results?.[0]?.envelope?.id, decisionId: decision.id };
}

async function screenshot(page, name) {
  await page.waitForLoadState('networkidle', { timeout: 3_000 }).catch(() => undefined);
  await page.waitForTimeout(350);
  await page.locator('body').waitFor({ state: 'visible', timeout: 10_000 });
  await page.screenshot({ path: path.join(screenshotDir, name), fullPage: true });
}

async function clickIfVisible(page, text) {
  const locator = page.getByRole('button', { name: text }).first();
  if (await locator.isVisible().catch(() => false) && await locator.isEnabled().catch(() => false)) {
    await locator.click();
    await page.waitForLoadState('networkidle', { timeout: 3_000 }).catch(() => undefined);
  }
}

await fs.mkdir(screenshotDir, { recursive: true });
for (const entry of await fs.readdir(screenshotDir)) {
  if (entry.endsWith('.png')) {
    await fs.unlink(path.join(screenshotDir, entry));
  }
}
const seeded = await seedProductDemo();

const browser = await chromium.launch();
const context = await browser.newContext({ viewport: { width: 1440, height: 1000 }, deviceScaleFactor: 1 });
const page = await context.newPage();

await page.goto(`${frontendUrl}/login`);
await screenshot(page, '01-login.png');
await page.evaluate((token) => localStorage.setItem('airspace.token', token), seeded.token);
await page.goto(`${frontendUrl}/explorer`);
await page.locator('.board-title', { hasText: 'Mission Explorer' }).waitFor({ timeout: 10_000 });
await screenshot(page, '02-mission-explorer.png');

await page.getByRole('button', { name: /Agentic Ops/i }).click();
await page.getByRole('button', { name: /Run all agents/i }).click();
await page.getByRole('heading', { name: 'Findings' }).waitFor({ timeout: 10_000 });
await page.getByPlaceholder(/Ask against decision trace/i).fill('Why this reroute?');
await page.getByRole('button', { name: 'Ask trace' }).click();
await page.getByText('Trace Answer').waitFor({ timeout: 10_000 });
await screenshot(page, '03-agentic-ops.png');

await page.goto(`${frontendUrl}/missions/${seeded.missionId}`);
await clickIfVisible(page, 'Show On Map');
await screenshot(page, '04-mission-workspace.png');

await page.goto(`${frontendUrl}/missions/${seeded.missionId}/brief`);
await screenshot(page, '05-pilot-brief.png');

await page.goto(`${frontendUrl}/missions/${seeded.missionId}/reservations/${seeded.reservationId}`);
await clickIfVisible(page, 'Show On Map');
await screenshot(page, '06-reservation-sections.png');
await clickIfVisible(page, 'NOTAM');
await screenshot(page, '07-reservation-supplements.png');

await page.goto(`${frontendUrl}/deconfliction/${seeded.reservationId}`);
await screenshot(page, '08-deconfliction-review.png');

await page.goto(`${frontendUrl}/messages/${seeded.messageId}`);
await screenshot(page, '09-messaging.png');

await page.goto(`${frontendUrl}/feed/${seeded.feedId ?? ''}`);
await screenshot(page, '10-usns-feed.png');

await page.goto(`${frontendUrl}/decisions/${seeded.decisionId}`);
await screenshot(page, '11-decision-summary.png');
await clickIfVisible(page, 'TRACE');
await screenshot(page, '12-decision-trace.png');
await clickIfVisible(page, 'SUMMARY');
await clickIfVisible(page, 'Show On Map');
await clickIfVisible(page, 'MAP');
await screenshot(page, '13-decision-map.png');

await page.goto(`${frontendUrl}/notams`);
await screenshot(page, '14-notam-constraints.png');

await page.goto(`${frontendUrl}/weather`);
await page.locator('.map-layer-group').first().getByRole('button', { name: 'Weather', exact: true }).click({ timeout: 1_000 }).catch(() => undefined);
await page.locator('.weather-event-card', { hasText: 'Hurricane / Convection' }).first().click({ timeout: 1_000 }).catch(() => undefined);
await clickIfVisible(page, 'Affected Missions');
await page.locator('.guidance-card', { hasText: 'Convective SIGMET' }).first().click({ timeout: 1_000 }).catch(() => undefined);
await page.waitForTimeout(500);
await clickIfVisible(page, 'Fit Selected');
await screenshot(page, '15-weather-pirep.png');
await page.locator('.weather-event-drilldown').first().scrollIntoViewIfNeeded({ timeout: 1_000 }).catch(() => undefined);
await screenshot(page, '20-weather-pattern-events.png');
await page.locator('select[aria-label="Source family filter"]').selectOption('WEATHER', { timeout: 1_000 }).catch(() => undefined);
await page.locator('input[placeholder="min confidence %"]').fill('50', { timeout: 1_000 }).catch(() => undefined);
await clickIfVisible(page, 'Movement Projection');
await page.locator('.map-feature-browser').scrollIntoViewIfNeeded({ timeout: 1_000 }).catch(() => undefined);
await screenshot(page, '21-weather-pattern-map-filters.png');

await page.goto(`${frontendUrl}/config`);
await screenshot(page, '16-config-reference.png');
await page.getByRole('button', { name: 'System Configuration' }).click();
await page.getByText('Agentic Store').waitFor({ timeout: 10_000 });
await screenshot(page, '17-agentic-system.png');

await page.goto(`${frontendUrl}/search`);
await page.locator('input[placeholder^="Mission"]').fill('decision');
await page.waitForTimeout(500);
await screenshot(page, '18-search.png');

await page.goto(`${frontendUrl}/history`);
await screenshot(page, '19-history-audit.png');

await browser.close();
console.log(JSON.stringify({ screenshotDir, ...seeded }, null, 2));
