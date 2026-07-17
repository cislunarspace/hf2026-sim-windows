/** 一条发往 UE 的控制指令(assign/stop)。 */
export interface RenderControlCommand {
    event: 'assign' | 'stop';
    render_id: string;
    aircraft: string[];
}
/** RenderScheduler 依赖的最小 Redis 能力(注入,便于单测 mock)。 */
export interface RenderSchedulerDeps {
    /** publish 到指定频道(channel 参数避免硬编码,便于测试)。 */
    publish(channel: string, message: string): Promise<void>;
    /**
     * 订阅 sim:render_id,收到消息时回调。
     * 返回 unsubscribe 函数(stop 时调)。
     */
    subscribe(channel: string, onMessage: (msg: string) => void): Promise<() => void>;
    /** 可选日志。 */
    warn?(msg: string): void;
    /** 可选普通日志。 */
    info?(msg: string): void;
    /** 可选:render-ctl plan 提供的容量(renderId → maxAircraft)。覆盖 UE 上报值。 */
    capacityOverride?: Record<string, number>;
}
/** 在线 UE 渲染端的信息。 */
interface RendererInfo {
    renderId: string;
    maxAircraft: number;
}
/**
 * 把 aircraft 按 renderers 容量贪心分配(填满第一个再溢出到下一个)。
 * 不变性:每架飞机至多出现在一个 UE 的分配里。
 */
export declare function planAssignment(aircraft: string[], renderers: RendererInfo[]): Map<string, string[]>;
/**
 * 有状态分配器:跟踪 pending 飞机池 + 各 UE 容量 + renderId↔pid 映射。
 *
 * 事件流:
 *   start(gimbalUavs)      → 飞机入 pending 池 + 订阅 sim:render_id
 *   UE online              → addRenderer + 分配 pending → publish assign
 *   UE offline / crash     → removeRenderer → 收回飞机 → 重分配 → publish assign
 *   stop()                 → unsubscribe + 清空状态(不主动发 stop —— UE 进程被 kill)
 */
export declare class RenderScheduler {
    private deps;
    /** renderId → 容量信息(在线 UE)。 */
    private renderers;
    /** pid → renderId(UE 进程崩溃时反查)。 */
    private pidToRenderId;
    /** 待分配飞机(无 UE 承接或被收回)。 */
    private pending;
    /** 已分配:renderId → aircraft[](stop/重分配时增量操作)。 */
    private assigned;
    /** 超出 UE 渲染容量、需要发 stop 指令的 aircraft ID。 */
    private excess;
    /** subscribe 返回的 unsubscribe 句柄。 */
    private unsub;
    private started;
    constructor(deps: RenderSchedulerDeps);
    /** 启动:记录待渲染飞机池,订阅 sim:render_id。等 UE 上线后自动分配。 */
    start(aircraft: string[], excessUavs?: string[]): Promise<void>;
    /** 注册一个 UE 进程(spawn 时调用,建立 pid↔renderId 映射)。 */
    registerUeProcess(pid: number, renderId: string): void;
    /** UE 进程崩溃(由 watchExit 触发)。按 pid 反查 renderId 并收回飞机重分配。 */
    onUeCrash(pid: number): Promise<void>;
    /** 处理 sim:render_id 频道的消息(renderer_online / renderer_offline)。 */
    onMessage(raw: string): Promise<void>;
    /** 停止:unsubscribe + 清空状态。不主动发 stop(UE 进程会被 manager kill)。 */
    stop(): Promise<void>;
    /** UE 上线:记录容量(优先 capacityOverride),分配 pending 飞机,对超出容量的 aircraft 发 stop。 */
    private addRenderer;
    /** UE 下线/崩溃:收回其飞机入 pending,重分配给其他 UE,publish stop+assign。 */
    private removeRenderer;
    /** 把 pending 飞机按各 UE 容量贪心分配,publish assign 指令。 */
    private drainPending;
    private publish;
    private warn;
    private info;
}
export {};
//# sourceMappingURL=render-scheduler.d.ts.map