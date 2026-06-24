# Collaborative Decision Workflow

Airspace models a local, human-reviewed collaborative decision workflow for weather, route, TMI, NOTAM, CARF/ALTRV, and pilot-brief coordination. It is meant to support simulation, public review, and future authorized integration. It is not an official FAA CDM system and does not transmit external messages.

## Public Research Basis

- FAA CDM is described as a process for ATFM decisions through increased information exchange among stakeholders, with stakeholder needs considered and better situational awareness: <https://tfmlearning.faa.gov/tfm-training/atfm-basics/cdm-t1-lesson1b.html>
- FAA FMDS describes collaboration with FAA stakeholders and airspace users, balancing demand and capacity, and supporting collaborative decision making around optimal routing: <https://www.faa.gov/air_traffic/technology/fmds>
- FAA TMI guidance says shared FCA/FEA data improves coordination and describes proposal, justification, joint review, approval authority, monitoring, adjustment, and cancellation concepts for TMIs: <https://www.faa.gov/air_traffic/publications/atpubs/foa_html/chap18_section_7.html>
- FAA/Volpe NAS Common Reference material describes the need to correlate SWIM data spatially and temporally so users can access the real-time data needed for decision making: <https://www.volpe.dot.gov/news/developing-newest-service-faa-swim-nas-common-reference>

## Implemented Local Model

| Concept | Airspace model |
|---|---|
| Stakeholder / participant | `CollaborativeParticipantSummary` |
| Common operating picture | `CommonOperatingPictureSummary` |
| Proposal | `CollaborativeProposalSummary` |
| Proposal request | `CollaborativeProposalRequest` |
| Comment | `CollaborativeCommentSummary` |
| Accept / reject / approve action | `CollaborativeApprovalSummary` |
| Delivery receipt | `CollaborativeDeliveryReceiptSummary` |

The local workflow supports these states:

1. `PROPOSED`
2. `ACCEPTED`
3. `REJECTED`
4. `APPROVED_LOCAL`
5. `DELIVERED_BY_OPERATOR`

Acceptance is a stakeholder response. `APPROVED_LOCAL` is the local human-approval gate. `DELIVERED_BY_OPERATOR` records an operator-entered receipt, but `externalSendPerformed=false` remains explicit until a future authorized delivery adapter exists.

## API Surface

```http
GET  /api/collaboration/common-operating-picture
GET  /api/collaboration/participants
GET  /api/collaboration/proposals
POST /api/collaboration/proposals
POST /api/collaboration/proposals/{proposalId}/comment
POST /api/collaboration/proposals/{proposalId}/accept
POST /api/collaboration/proposals/{proposalId}/reject
POST /api/collaboration/proposals/{proposalId}/approve
POST /api/collaboration/proposals/{proposalId}/deliver
```

`CommonOperatingPictureSummary` currently aggregates local mission count, affected mission count, provider status/freshness, proposal count, pending approval count, delivered receipt count, participant roles, proposals, source refs, and diagnostics.

## Non-Goals

- No official FAA CDM state synchronization.
- No live SWIM/NCR/FMDS/TFMS data exchange.
- No automatic official workflow mutation.
- No external USNS/NADIN/WMSCR/KVM/SWIM delivery.
- No claim that local proposal approval is FAA/ATCSCC approval.

The next authorized-integration step is to map real provider receipts and stakeholder directories into these DTOs while preserving source mode, consent scope, egress policy, human approval, and delivery receipt boundaries.
