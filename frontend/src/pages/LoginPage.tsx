import { useForm } from 'react-hook-form';
import { useNavigate } from 'react-router-dom';
import { z } from 'zod';
import { api } from '../api/client';
import { useSession } from '../state/session';

const schema = z.object({ username: z.string().min(1), password: z.string().min(1) });
type FormData = z.infer<typeof schema>;

export function LoginPage() {
  const navigate = useNavigate();
  const setSession = useSession((state) => state.setSession);
  const { register, handleSubmit, formState, setError } = useForm<FormData>({
    defaultValues: { username: 'planner', password: 'planner' }
  });
  const onSubmit = handleSubmit(async (values) => {
    const parsed = schema.safeParse(values);
    if (!parsed.success) return;
    try {
      const result = await api.login(values.username, values.password);
      if (!result.accepted || !result.token || !result.user) {
        setError('password', { message: result.diagnostics?.join(', ') || 'Invalid login' });
        return;
      }
      setSession(result.token, result.user);
      navigate('/explorer', { replace: true });
    } catch (error) {
      setError('password', {
        message: error instanceof Error ? error.message : 'Unable to reach the Airspace API'
      });
    }
  });
  return (
    <main className="login-screen">
      <form className="login-panel" onSubmit={onSubmit}>
        <h1>Airspace Operations</h1>
        <label>Username<input autoComplete="username" {...register('username')} /></label>
        <label>Password<input type="password" autoComplete="current-password" {...register('password')} /></label>
        {formState.errors.password && <p className="error">{formState.errors.password.message}</p>}
        <button type="submit">Sign in</button>
      </form>
    </main>
  );
}
