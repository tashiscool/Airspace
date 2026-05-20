export function StatusBadge({ value }: { value?: string | boolean }) {
  const text = String(value ?? 'UNKNOWN');
  const tone = text === 'true' || ['APPROVED', 'CLEAR', 'ACCEPTED'].includes(text) ? 'good'
    : ['BLOCKED', 'REJECTED', 'CANCELLED'].includes(text) ? 'bad'
      : 'warn';
  return <span className={`badge ${tone}`}>{text}</span>;
}
