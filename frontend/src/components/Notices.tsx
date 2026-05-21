import type { UseMutationResult, UseQueryResult } from '@tanstack/react-query';

export function ErrorNotice({ error, title = 'Unable to load data' }: { error?: unknown; title?: string }) {
  if (!error) return null;
  return (
    <div className="notice error-notice" role="alert">
      <strong>{title}</strong>
      <span>{errorMessage(error)}</span>
    </div>
  );
}

export function QueryNotice<TData>({ query, label }: { query: UseQueryResult<TData, Error>; label: string }) {
  if (query.isLoading) {
    return <div className="notice loading-notice"><strong>{label}</strong><span>Loading...</span></div>;
  }
  return <ErrorNotice error={query.error} title={`${label} unavailable`} />;
}

export function MutationNotice<TData, TVariables>({ mutation, label }: { mutation: UseMutationResult<TData, Error, TVariables, unknown>; label: string }) {
  if (mutation.isPending) {
    return <div className="notice loading-notice"><strong>{label}</strong><span>Working...</span></div>;
  }
  return <ErrorNotice error={mutation.error} title={`${label} failed`} />;
}

export function errorMessage(error: unknown) {
  if (error instanceof Error) return error.message;
  if (typeof error === 'string') return error;
  return 'An unexpected error occurred.';
}
