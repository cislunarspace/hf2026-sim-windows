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
    /**
     * 探测进程是否仍存活(kill(pid,0) 语义)。exit 事件可能因 detached+unref
     * 丢失,watchdog 用此方法兜底检测 competition 真正退出。
     * exited 已置 true 时返回 false;否则向 pid 发信号 0 探测。
     */
    isAlive(): boolean;
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
    photoMode?: 'auto' | 'on' | 'off';
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
    /** 036: 正在等待仿真 ready 帧 / 首个 sim:state，期间保持 loading。 */
    private waitingForReady;
    /** 036: sim:progress ready 帧订阅的取消函数。 */
    private readyCleanup;
    /** 036: ready 帧超时定时器。 */
    private readyTimeout;
    /**
     * 进程存活 watchdog 定时器:每 2 秒探测 competition 进程是否仍存活。
     * 兜底 child exit 事件因 detached+unref 丢失的情况(实测 competition
     * 10 分钟自然结束后 exit 事件未触发,UE 成孤儿)。检测到进程死亡即触发清理。
     */
    private watchdogTimer;
    constructor(deps: SimManagerDeps);
    getState(): SessionState;
    /** 订阅 sim:state 频道,感知外部启动的仿真(命令行启动)。 */
    private subscribeToStateChannel;
    /** 处理 sim:state 消息,更新内部状态以反映外部启动的仿真。 */
    private handleStateChannelMessage;
    /** 所有状态变更的唯一出口:更新 + 广播 + 去重。 */
    private setState;
    /** 036: 订阅 sim:progress 的 ready/就绪 帧;收到即切 running。 */
    private subscribeToReadyProgress;
    /** 036: 取消 ready 订阅与超时,切到 running(幂等)。 */
    private markReady;
    /** 036: 清理 ready 相关订阅与超时(进程退出 / stop 时调用)。 */
    private cleanupReady;
    /**
     * 启动 competition 进程(单一子进程)。
     * competition 内部自己 spawn opensim-sim 并轮询就绪,bridge 不重复做。
     */
    start(sc: StartScenario, opts: StartOptions): Promise<SessionState>;
    /**
     * Spec 028: 调 render-ctl plan → spawn UE 进程 → 装配 RenderScheduler。
     * 全程 best-effort:失败只 WARN,不抛错(仿真照跑)。
     * UE stdout/stderr 重定向到 outDir/ue_<rendererId>.log。
     */
    private startRenderers;
    /** spawn 单个 UE 实例(按 plan 的 argv/cwd/env)。失败返回 null(降级)。 */
    private spawnUe;
    private warn;
    /** 注册子进程退出监听:非 stop 上下文退出 → error。 */
    private watchExit;
    /**
     * competition 退出的统一清理路径(由 exit 事件或 watchdog 触发)。
     * code=null 表示 watchdog 探测到进程消失但未收到 exit 事件(按非 0 处理)。
     *
     * 关键:无论 session 当前状态如何,只要 competition 进程没了就必杀 UE。
     * 旧逻辑的 `status === idle/stopping` guard 会在 sim:state 提前把状态切到
     * idle 时跳过清理,导致 UE 孤儿。孤儿 UE 占 GPU/CPU 远比重复清理危害大。
     */
    private handleCompetitionExit;
    /**
     * 启动 competition 存活 watchdog(spawn 后调用)。
     * 兜底 child exit 事件因 detached+unref 丢失的情况:每 2 秒用
     * kill(pid,0) 探测 competition 进程,消失即触发清理。
     */
    private startWatchdog;
    private stopWatchdog;
    private watchdogTick;
    /** 暂停:发 pause 命令给引擎(competition 主循环空转检测自动跟随)。 */
    pause(): Promise<SessionState>;
    resume(): Promise<SessionState>;
    /** 关闭:发 end → 停 RenderScheduler → 两段式 kill UE → kill competition → 回 idle。幂等。 */
    stop(): Promise<SessionState>;
    /** 停止订阅 sim:state 频道(bridge 停止时调用)。 */
    unsubscribe(): Promise<void>;
    /**
     * Spec 028: 两段式 kill 所有 UE 进程(SIGTERM → stopGrace → SIGKILL)。
     *
     * 关键:bridge spawn 的是 `bash run.sh`,run.sh 再 spawn testwl.sh,testwl.sh
     * 再 spawn 真正的 UE 二进制。bash 收到 SIGTERM 退出后,其 exit 事件触发
     * (`exited=true`),但**孙子进程 UE 仍在同一进程组里活着**(被 init 收养)。
     * 旧逻辑用 `exited` 判断是否要 SIGKILL → 只看 bash,误以为全干净,UE 成孤儿。
     *
     * 修复:SIGTERM 后用 isAlive() 探测(而非 exited),且 SIGKILL 阶段对每个曾
     * spawn 的进程**无条件**发信号(process.kill(-pgid) 给整个进程组,杀掉所有
     * 后代无论多深,无论 bash 是否已 exited)。
     */
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