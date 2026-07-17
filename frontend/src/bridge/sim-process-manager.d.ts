import { type ExecFileFn } from './render-ctl-client';
export type SessionStatus = 'idle' | 'starting' | 'loading' | 'running' | 'paused' | 'stopping' | 'error';
export interface SessionState {
    status: SessionStatus;
    /** 当前运行的 scenario id(null = 无会话)。 */
    scenario: string | null;
    sessionId: string | null;
    error: string | null;
}
/** 注入的子进程句柄(屏蔽 ChildProcess 细节,测试可 mock)。 */
export interface ManagedProcess {
    readonly pid: number;
    /** 是否已退出(exit 事件触发后置 true)。 */
    readonly exited: boolean;
    /** 注册退出回调(仅触发一次)。 */
    onExit(cb: (code: number | null) => void): void;
    /** 发送信号。Windows 上 SIGTERM/SIGKILL 均强制终止。 */
    kill(signal: string): boolean;
}
export interface SimManagerDeps {
    spawn: (cmd: string, args: string[], opts?: {
        cwd?: string;
        logFile?: string;
        env?: NodeJS.ProcessEnv;
        detached?: boolean;
    }) => ManagedProcess;
    redis: {
        publish(channel: string, message: string): Promise<void>;
        subscribe?(channel: string, onMessage: (msg: string) => void): Promise<() => void>;
    };
    execFile?: ExecFileFn;
    redisHost?: string;
    redisPort?: number;
    stopGrace: number;
    commandChannel: string;
    stateChannel: string;
    onStateChange: (s: SessionState) => void;
    sleep: (ms: number) => Promise<void>;
    now: () => number;
    makeSessionId: () => string;
    warn?: (msg: string) => void;
}
/** discovery 注册表里的场景条目(传给 start 的第一个参数)。 */
export interface StartScenario {
    id: string;
    baselineAgent: string;
    defaultDuration: number;
    scenarioJson: string;
    /** 选手自定义算法（'module:Class'）；优先于 baselineAgent。undefined 则用 baseline。 */
    agent?: string;
    mode?: 'train' | 'eval';
    photo?: boolean;
    yoloModel?: string;
    accuracy?: number;
    noiseSigma?: number;
    routeSeed?: number;
}
export interface StartOptions {
    pythonBin: string;
    scenariosDir: string;
    renderCtlBinary?: string;
    renderersDir?: string;
    /** scenario.json 绝对路径(render-ctl plan --config 需要)。由 endpoint 计算。 */
    scenarioJsonAbs: string;
}
export declare class SimProcessManager {
    private state;
    private competitionProc;
    private stopRequested;
    private deps;
    private ueProcs;
    private renderScheduler;
    /** spawn 后到 scheduler 装配前的 pid→rendererId 暂存(scheduler 就绪后注册)。 */
    private renderSchedulerPending;
    /** sim:state 订阅取消函数(bridge 停止时调用)。 */
    private unsubscribeStateChannel;
    /** 是否已通过 bridge 启动仿真(区别于外部命令行启动)。 */
    private startedByBridge;
    constructor(deps: SimManagerDeps);
    getState(): SessionState;
    /** 订阅 sim:state 频道,感知外部启动的仿真(命令行启动)。 */
    private subscribeToStateChannel;
    /** 处理 sim:state 消息,更新内部状态以反映外部启动的仿真。 */
    private handleStateChannelMessage;
    /** 所有状态变更的唯一出口:更新 + 广播 + 去重。 */
    private setState;
    /**
     * 启动 competition 进程(单一子进程)。
     * competition 内部自己 spawn opensim-sim 并轮询就绪,bridge 不重复做。
     */
    start(sc: StartScenario, opts: StartOptions): Promise<SessionState>;
    /**
     * Spec 028: 调 render-ctl plan → spawn UE 进程 → 装配 RenderScheduler。
     * 全程 best-effort:失败只 WARN,不抛错(仿真照跑)。
     */
    private startRenderers;
    /** spawn 单个 UE 实例(按 plan 的 argv/cwd/env)。失败返回 null(降级)。 */
    private spawnUe;
    private warn;
    /** 注册子进程退出监听:非 stop 上下文退出 → error。 */
    private watchExit;
    /** 暂停:发 pause 命令给引擎(competition 主循环空转检测自动跟随)。 */
    pause(): Promise<SessionState>;
    resume(): Promise<SessionState>;
    /** 关闭:发 end → 停 RenderScheduler → 两段式 kill UE → kill competition → 回 idle。幂等。 */
    stop(): Promise<SessionState>;
    /** 停止订阅 sim:state 频道(bridge 停止时调用)。 */
    unsubscribe(): Promise<void>;
    /** Spec 028: 两段式 kill 所有 UE 进程(SIGTERM → stopGrace → SIGKILL)。 */
    private killUeProcs;
    /** 两段式:SIGTERM → stopGrace → SIGKILL。 */
    private killProc;
}
/** 包装 child_process.spawn 为 ManagedProcess。 */
export declare function spawnChildProcess(cmd: string, args: string[], opts?: {
    cwd?: string;
    logFile?: string;
    env?: NodeJS.ProcessEnv;
    detached?: boolean;
}): ManagedProcess;
/** 生产环境依赖装配(ioredis publish)。 */
export declare function createProductionDeps(params: {
    redisHost: string;
    redisPort: number;
    stopGrace: number;
    commandChannel: string;
    stateChannel: string;
    onStateChange: (s: SessionState) => void;
}): SimManagerDeps;
//# sourceMappingURL=sim-process-manager.d.ts.map