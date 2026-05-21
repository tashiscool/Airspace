export function StatusBadge({ value }: { value?: string | boolean }) {
  const text = String(value ?? 'UNKNOWN').toUpperCase();
  const tone = text === 'TRUE' || ['APPROVED', 'CLEAR', 'ACCEPTED', 'VALIDATED', 'COMPLETE', 'COMPLETED', 'SENT', 'ACK'].includes(text) ? 'good'
    : ['BLOCKED', 'REJECTED', 'CANCELLED', 'FAILED', 'ERROR', 'CONFLICT', 'CONFLICTS'].includes(text) ? 'bad'
      : 'warn';
  return <span className={`badge ${tone}`}>{text}</span>;
}
