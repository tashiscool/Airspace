export function shouldWarnForNavigation(hasUnsavedChanges: boolean, currentLocation: string, targetHref: string, origin = 'http://localhost') {
  if (!hasUnsavedChanges) return false;
  try {
    const target = new URL(targetHref, origin);
    const current = new URL(currentLocation, origin);
    if (target.origin !== current.origin) return false;
    return `${target.pathname}${target.search}${target.hash}` !== `${current.pathname}${current.search}${current.hash}`;
  } catch {
    return false;
  }
}

export function isPlainLeftClick(event: MouseEvent) {
  return event.button === 0 && !event.metaKey && !event.ctrlKey && !event.altKey && !event.shiftKey;
}
