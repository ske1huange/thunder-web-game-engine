type Listener<T> = (event: T) => void;

/** 带类型约束的简单事件派发器 */
export class EventEmitter<Events extends object> {
  private readonly listeners = new Map<keyof Events, Set<Listener<never>>>();

  on<K extends keyof Events>(type: K, listener: Listener<Events[K]>): () => void {
    let set = this.listeners.get(type);
    if (!set) {
      set = new Set();
      this.listeners.set(type, set);
    }
    set.add(listener as Listener<never>);
    return () => this.off(type, listener);
  }

  once<K extends keyof Events>(type: K, listener: Listener<Events[K]>): () => void {
    const off = this.on(type, (e) => {
      off();
      listener(e);
    });
    return off;
  }

  off<K extends keyof Events>(type: K, listener: Listener<Events[K]>): void {
    this.listeners.get(type)?.delete(listener as Listener<never>);
  }

  hasListeners<K extends keyof Events>(type: K): boolean {
    const set = this.listeners.get(type);
    return !!set && set.size > 0;
  }

  emit<K extends keyof Events>(type: K, event: Events[K]): void {
    const set = this.listeners.get(type);
    if (!set) return;
    for (const l of [...set]) (l as Listener<Events[K]>)(event);
  }

  removeAllListeners(): void {
    this.listeners.clear();
  }
}
