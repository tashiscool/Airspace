import type { MessageSummary } from '../types';

export const DEFAULT_RECIPIENT_PRESETS = [
  'CARF',
  'USNOF',
  'KDZZNAXX',
  'KZNY',
  'KZDC',
  'KZLA',
  'KZMA',
  'KZBW'
];

export function normalizeRecipients(value: string) {
  return value
    .split(/[,\n;]/)
    .map((item) => item.trim().toUpperCase())
    .filter(Boolean)
    .filter((item, index, all) => all.indexOf(item) === index);
}

export function messageBodyWithMetadata(rawText: string, recipients: string, attachments: string) {
  const trimmedRecipients = normalizeRecipients(recipients).join(', ');
  const trimmedAttachments = attachments.split(/[,\n;]/).map((item) => item.trim()).filter(Boolean).join(', ');
  return [
    trimmedRecipients ? `TO: ${trimmedRecipients}` : undefined,
    trimmedAttachments ? `ATTACHMENTS: ${trimmedAttachments}` : undefined,
    rawText
  ].filter(Boolean).join('\n');
}

export function relatedMessages(messages: MessageSummary[], selected: MessageSummary) {
  return messages
    .filter((message) => message.id !== selected.id)
    .filter((message) =>
      (selected.reservationId && message.reservationId === selected.reservationId)
      || (selected.missionId && message.missionId === selected.missionId)
      || message.family === selected.family
    )
    .slice(0, 8);
}

export function recipientPresetSuggestions(query: string, presets: string[] = DEFAULT_RECIPIENT_PRESETS) {
  const q = query.trim().toUpperCase();
  return presets.filter((preset) => !q || preset.includes(q)).slice(0, 8);
}
