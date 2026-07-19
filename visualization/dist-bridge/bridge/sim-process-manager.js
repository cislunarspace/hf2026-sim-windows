"use strict";
// 仿真会话子进程编排器 —— 单进程 competition 模型 + UE 渲染旁路(Spec 028)。
//
// 职责:
//   1. spawn `python -m competition run --scenario <id> --agent <baseline>
//      --start-sim` 单一进程(它内部自己 spawn opensim-sim 并轮询就绪);
//   2. [Spec 028] 若 renderCtlBinary 提供:调 opensim-render-ctl plan → 按 plan
//      spawn UE 渲染进程(GPU 隔离 + detached)→ 装配 RenderScheduler 接收
//      renderer_online 并分配飞机。任一步失败 → WARN 降级,仿真照跑。
//   3. pause/resume(向 sim:commands 发 {cmd:pause/resume});
//   4. stop(发 {cmd:end} + 停 RenderScheduler + 两段式 kill UE + kill competition)。
//
// 渲染是 bridge 旁路:competition 不感知 UE;UE 崩溃不影响仿真。
// 可测性:spawn / execFile / redis / subscribe / 时间 / sleep 均经 SimManagerDeps 注入。
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || (function () {
    var ownKeys = function(o) {
        ownKeys = Object.getOwnPropertyNames || function (o) {
            var ar = [];
            for (var k in o) if (Object.prototype.hasOwnProperty.call(o, k)) ar[ar.length] = k;
            return ar;
        };
        return ownKeys(o);
    };
    return function (mod) {
        if (mod && mod.__esModule) return mod;
        var result = {};
        if (mod != null) for (var k = ownKeys(mod), i = 0; i < k.length; i++) if (k[i] !== "default") __createBinding(result, mod, k[i]);
        __setModuleDefault(result, mod);
        return result;
    };
})();
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.SimProcessManager = void 0;
exports.spawnChildProcess = spawnChildProcess;
exports.createProductionDeps = createProductionDeps;
const child_process_1 = require("child_process");
const fs = __importStar(require("fs"));
const path = __importStar(require("path"));
const ioredis_1 = __importDefault(require("ioredis"));
const render_ctl_client_1 = require("./render-ctl-client");
const render_scheduler_1 = require("./render-scheduler");
function scenarioOutputDir(scenariosDir, scenarioJson) {
    return path.join(scenariosDir, path.dirname(scenarioJson), 'output');
}
// ── SimProcessManager ─────────────────────────────────────────────────
class SimProcessManager {
    constructor(deps) {
        this.state = { status: 'idle', scenario: null, sessionId: null, error: null };
        this.competitionProc = null;
        this.stopRequested = false;
        // Spec 028: UE 渲染进程表(spawn 后跟踪,stop 时 kill;崩溃触发 onUeCrash)。
        this.ueProcs = [];
        this.renderScheduler = null;
        /** spawn 后到 scheduler 装配前的 pid→rendererId 暂存(scheduler 就绪后注册)。 */
        this.renderSchedulerPending = [];
        /** sim:state 订阅取消函数(bridge 停止时调用)。 */
        this.unsubscribeStateChannel = null;
        /** 是否已通过 bridge 启动仿真(区别于外部命令行启动)。 */
        this.startedByBridge = false;
        /** 036: 正在等待仿真 ready 帧 / 首个 sim:state，期间保持 loading。 */
        this.waitingForReady = false;
        /** 036: sim:progress ready 帧订阅的取消函数。 */
        this.readyCleanup = null;
        /** 036: ready 帧超时定时器。 */
        this.readyTimeout = null;
        this.deps = deps;
        this.subscribeToStateChannel();
    }
    getState() {
        return { ...this.state };
    }
    /** 订阅 sim:state 频道,感知外部启动的仿真(命令行启动)。 */
    async subscribeToStateChannel() {
        if (!this.deps.redis.subscribe)
            return;
        try {
            this.unsubscribeStateChannel = await this.deps.redis.subscribe(this.deps.stateChannel, (msg) => this.handleStateChannelMessage(msg));
        }
        catch (e) {
            this.warn(`failed to subscribe to ${this.deps.stateChannel}: ${e.message}`);
        }
    }
    /** 处理 sim:state 消息,更新内部状态以反映外部启动的仿真。 */
    handleStateChannelMessage(msg) {
        try {
            const parsed = JSON.parse(msg);
            const simStatus = parsed.status;
            if (!simStatus)
                return;
            if (this.startedByBridge) {
                // 036: bridge 启动的会话不依赖首个 sim:state 切 running——那发生在
                // controller 做路径规划之前,会过早收起进度条。真实就绪以 controller 的
                // '就绪' 进度帧为准(见 subscribeToReadyProgress); 进程崩溃/超时兜底。
                return;
            }
            if (this.state.status === 'idle') {
                if (simStatus === 'running') {
                    this.state = {
                        status: 'running',
                        scenario: null,
                        sessionId: `external_${Date.now().toString(36)}`,
                        error: null,
                    };
                    this.deps.onStateChange(this.getState());
                }
                else if (simStatus === 'paused') {
                    this.state = {
                        status: 'paused',
                        scenario: null,
                        sessionId: `external_${Date.now().toString(36)}`,
                        error: null,
                    };
                    this.deps.onStateChange(this.getState());
                }
            }
            else if (this.state.status === 'running' || this.state.status === 'paused') {
                if (simStatus === 'ended' || simStatus === 'idle') {
                    this.state = {
                        status: 'idle',
                        scenario: null,
                        sessionId: null,
                        error: null,
                    };
                    this.deps.onStateChange(this.getState());
                }
                else if (simStatus === 'running' || simStatus === 'paused') {
                    if (this.state.status !== simStatus) {
                        this.state = { ...this.state, status: simStatus };
                        this.deps.onStateChange(this.getState());
                    }
                }
            }
        }
        catch {
            // 忽略解析错误
        }
    }
    /** 所有状态变更的唯一出口:更新 + 广播 + 去重。 */
    setState(next, error = null) {
        if (this.state.status === next && this.state.error === error)
            return;
        this.state = { ...this.state, status: next, error };
        this.deps.onStateChange(this.getState());
    }
    /** 036: 订阅 sim:progress 的 ready/就绪 帧;收到即切 running。 */
    async subscribeToReadyProgress() {
        if (!this.deps.redis.subscribe)
            return;
        try {
            const unsub = await this.deps.redis.subscribe('sim:progress', (msg) => {
                try {
                    const parsed = JSON.parse(msg);
                    // 036: 只有 Python controller 的最终就绪帧(phase='就绪')才切 running。
                    // 引擎自身的 'ready' 只是进度节点,此时 controller 还在做路径规划,
                    // 不应过早把 session 置为 running(否则前端会提前收起进度条)。
                    if (parsed.type === 'load_progress' && parsed.phase === '就绪') {
                        this.warn('[SimProcessManager] controller ready frame received; transitioning to running');
                        this.markReady();
                    }
                }
                catch { /* ignore parse errors */ }
            });
            this.readyCleanup = () => {
                try {
                    unsub();
                }
                catch { /* ignore */ }
            };
        }
        catch (e) {
            this.warn(`failed to subscribe to sim:progress: ${e.message}`);
        }
    }
    /** 036: 取消 ready 订阅与超时,切到 running(幂等)。 */
    markReady() {
        if (!this.waitingForReady)
            return;
        this.waitingForReady = false;
        if (this.readyTimeout) {
            clearTimeout(this.readyTimeout);
            this.readyTimeout = null;
        }
        if (this.readyCleanup) {
            this.readyCleanup();
            this.readyCleanup = null;
        }
        if (this.state.status === 'loading' || this.state.status === 'starting') {
            this.setState('running');
        }
    }
    /** 036: 清理 ready 相关订阅与超时(进程退出 / stop 时调用)。 */
    cleanupReady() {
        this.waitingForReady = false;
        if (this.readyTimeout) {
            clearTimeout(this.readyTimeout);
            this.readyTimeout = null;
        }
        if (this.readyCleanup) {
            this.readyCleanup();
            this.readyCleanup = null;
        }
    }
    /**
     * 启动 competition 进程(单一子进程)。
     * competition 内部自己 spawn opensim-sim 并轮询就绪,bridge 不重复做。
     */
    async start(sc, opts) {
        // error 态允许重新开始:先清理残留进程(含 UE),再回 idle。
        if (this.state.status === 'error') {
            if (this.renderScheduler) {
                await this.renderScheduler.stop().catch(() => { });
                this.renderScheduler = null;
            }
            await this.killUeProcs();
            await this.killProc();
            this.competitionProc = null;
            this.state = { status: 'idle', scenario: null, sessionId: null, error: null };
            this.deps.onStateChange(this.getState());
        }
        if (this.state.status !== 'idle') {
            throw new Error('session_already_active');
        }
        this.stopRequested = false;
        this.startedByBridge = true;
        const sessionId = this.deps.makeSessionId();
        this.state = { status: 'starting', scenario: sc.id, sessionId, error: null };
        this.deps.onStateChange(this.getState());
        // competition 需从 repo 根运行(找 competition 包 + build/ 下的 sim)。
        // scenariosDir = <repo>/competition/scenarios,故 repo 根上溯两级。
        const repoRoot = path.dirname(path.dirname(opts.scenariosDir));
        const args = [
            '-m', 'competition', 'run',
            '--scenario', sc.id,
            '--agent', sc.agent || sc.baselineAgent,
            '--start-sim',
            '--duration', String(sc.defaultDuration),
            '--redis-host', this.deps.redisHost ?? '127.0.0.1',
            '--redis-port', String(this.deps.redisPort ?? 6379),
        ];
        // Spec 033: 感知参数条件追加（undefined → 不追加，命令行与改造前逐字节一致）。
        if (sc.mode)
            args.push('--mode', sc.mode);
        if (sc.photo)
            args.push('--photo');
        if (sc.yoloModel)
            args.push('--yolo-model', sc.yoloModel);
        // 防泄漏钳制后的 accuracy/noise（已由 endpoint 限定 [0,0.9] / ≥30）。
        if (sc.accuracy !== undefined)
            args.push('--accuracy', String(sc.accuracy));
        if (sc.noiseSigma !== undefined)
            args.push('--noise-sigma', String(sc.noiseSigma));
        // 路线种子: 正整数才透传(0 = 不随机,后端默认行为)。
        if (sc.routeSeed && sc.routeSeed > 0)
            args.push('--seed', String(sc.routeSeed));
        const outDir = scenarioOutputDir(opts.scenariosDir, sc.scenarioJson);
        // 把 outDir 显式传给 controller: 让 prepared.json / controller.stderr.log /
        // sim.stderr.log 都写到同一目录(否则 controller 用默认 output='output' 写到
        // cwd=repoRoot 下的 output/,而 bridge 期望 competition/scenarios/.../output/)。
        args.push('--output', outDir);
        try {
            this.competitionProc = this.deps.spawn(opts.pythonBin, args, {
                cwd: repoRoot,
                logFile: path.join(outDir, 'controller.stderr.log'),
                env: {
                    ...process.env,
                    OPENSIM_SIM_STDERR: path.join(outDir, 'sim.stderr.log'),
                },
                detached: true,
            });
        }
        catch (e) {
            this.setState('error', 'competition_spawn_failed');
            throw e;
        }
        this.setState('loading');
        this.watchExit(this.competitionProc, 'competition_crashed');
        // 036: 在 spawn 后保持 loading;监听 sim:progress ready 帧或首个 sim:state
        // 才认为仿真真正运行。进程崩溃由 watchExit 兜底。
        this.waitingForReady = true;
        await this.subscribeToReadyProgress();
        this.readyTimeout = setTimeout(() => {
            if (this.waitingForReady) {
                this.warn('[SimProcessManager] ready timeout after 5 minutes; marking error');
                this.waitingForReady = false;
                this.readyCleanup?.();
                this.readyCleanup = null;
                this.setState('error', 'ready_timeout');
            }
        }, 5 * 60 * 1000);
        // 启动后短暂等待;进程立即退出 → crashed。
        await this.deps.sleep(50);
        if (this.competitionProc.exited) {
            this.setState('error', 'competition_crashed');
            throw new Error('competition_crashed');
        }
        // Spec 028: UE 渲染旁路。renderCtlBinary 缺省 → 跳过(仿真照跑)。
        // 任一步失败 → WARN 降级,不影响 competition 已起来的会话。
        // 036: 渲染启动完成后仍保持 loading,由 sim:progress ready / sim:state 触发 running。
        await this.startRenderers(opts).catch((e) => {
            this.warn(`renderer orchestration degraded: ${e.message}`);
        });
        return this.getState();
    }
    /**
     * Spec 028: 调 render-ctl plan → spawn UE 进程 → 装配 RenderScheduler。
     * 全程 best-effort:失败只 WARN,不抛错(仿真照跑)。
     */
    async startRenderers(opts) {
        if (!opts.renderCtlBinary)
            return; // 渲染子系统休眠
        if (!this.deps.execFile) {
            this.warn('execFile not configured, skipping UE spawn');
            return;
        }
        if (!this.deps.redis.subscribe) {
            this.warn('redis.subscribe not available, skipping UE spawn');
            return;
        }
        let plan;
        try {
            plan = await (0, render_ctl_client_1.planRenderers)({
                execFile: this.deps.execFile,
                renderCtlBinary: opts.renderCtlBinary,
                scenarioJsonAbs: opts.scenarioJsonAbs,
                renderersDir: opts.renderersDir ?? 'config/renderers',
            });
        }
        catch (e) {
            this.warn(`render-ctl plan failed: ${e.message}`);
            return;
        }
        if (plan.skipped.length > 0) {
            this.warn(`render-ctl skipped: ${plan.skipped.join('; ')}`);
        }
        if (plan.instances.length === 0) {
            this.warn('no UE instances to spawn (gimbal UAVs empty or GPU insufficient)');
            return;
        }
        // ── 同步 scenario.json 到 UE 包目录 ───────────────────────────────
        // UE 启动时读 <workdir>/testwl/Content/Config/scenario.json 获知场景实体
        // 初始位置(gimbal UAV / 目标 / 诱饵),不复制则 UE 无法产生 sync_camera 帧。
        // 多 instance 可能共享同一 workdir,去重后每个 workdir 覆盖一次。
        const ueScenarioRel = 'testwl/Content/Config/scenario.json';
        const coveredWorkdirs = new Set();
        for (const inst of plan.instances) {
            if (coveredWorkdirs.has(inst.cwd))
                continue;
            coveredWorkdirs.add(inst.cwd);
            const ueScenarioPath = path.join(inst.cwd, ueScenarioRel);
            try {
                fs.mkdirSync(path.dirname(ueScenarioPath), { recursive: true });
                fs.copyFileSync(opts.scenarioJsonAbs, ueScenarioPath);
                console.log(`[SimProcessManager] scenario.json synced to UE: ${ueScenarioPath}`);
            }
            catch (e) {
                this.warn(`failed to sync scenario.json to UE (${ueScenarioPath}): ${e.message}`);
            }
        }
        // 容量覆盖:plan 的 max_aircraft 优先(UE 上报值常硬编码不准)。
        const capacityOverride = {};
        for (const inst of plan.instances) {
            capacityOverride[inst.rendererId] = inst.maxAircraft;
        }
        // 先装配 RenderScheduler 并订阅 sim:render_id,再 spawn UE。否则 UE 启动
        // 很快时可能先发布 renderer_online,bridge 还没订阅就丢消息,后续永远
        // 不会 assign aircraft,导致 UE 已启动但不产出 sync_camera 帧。
        const schedDeps = {
            publish: (ch, msg) => this.deps.redis.publish(ch, msg),
            subscribe: (ch, cb) => this.deps.redis.subscribe(ch, cb),
            warn: (m) => this.warn(m),
            info: (m) => console.log(`[RenderScheduler] ${m}`),
            capacityOverride,
        };
        this.renderScheduler = new render_scheduler_1.RenderScheduler(schedDeps);
        await this.renderScheduler.start(plan.gimbalUavs, plan.excessUavs);
        const spawned = [];
        // spawn 各 UE 实例(detached + GPU 隔离 env)。
        for (const inst of plan.instances) {
            const proc = this.spawnUe(inst);
            if (proc) {
                this.ueProcs.push(proc);
                spawned.push(proc);
                // 崩溃兜底:用 plan 的 rendererId 做 pid 映射(UE 若用自生成 render_id
                // 上线,onUeCrash 的 removeRenderer 为 no-op 不误伤;飞机由后续 UE
                // online 或 stop 时清理)。stopRequested 时退出不触发(避免误报)。
                const pid = proc.pid;
                const rid = inst.rendererId;
                proc.onExit(() => {
                    if (this.stopRequested)
                        return;
                    this.renderScheduler?.onUeCrash(pid).catch(() => { });
                });
                // 注意:RenderScheduler 内部维护 pid→renderId 映射。spawn 后先注册,
                // 让 onUeCrash 能反查(rendererId 来自 plan;UE online 时若上报同 id 则
                // 精确匹配,否则尽力)。
                this.renderSchedulerPending.push({ pid, rendererId: rid });
            }
        }
        if (spawned.length === 0) {
            this.warn('all UE spawns failed');
            await this.renderScheduler.stop().catch(() => { });
            this.renderScheduler = null;
            return;
        }
        // spawn 时 pid 已知,但 UE 的 render_id 要等 online 才确定。这里用 plan 的
        // rendererId 注册 pid 映射(尽力崩溃兜底)。
        for (const { pid, rendererId } of this.renderSchedulerPending) {
            this.renderScheduler.registerUeProcess(pid, rendererId);
        }
        this.renderSchedulerPending = [];
    }
    /** spawn 单个 UE 实例(按 plan 的 argv/cwd/env)。失败返回 null(降级)。 */
    spawnUe(inst) {
        // plan 通常保证 argv[0] 存在(= 启动脚本/exe);缺失时按平台选默认 shell
        const defaultShell = process.platform === 'win32' ? 'cmd.exe' : '/bin/bash';
        const cmd = inst.argv[0] ?? defaultShell;
        const args = inst.argv.slice(1);
        try {
            return this.deps.spawn(cmd, args, {
                cwd: inst.cwd,
                env: inst.env,
                detached: true,
            });
        }
        catch (e) {
            this.warn(`UE spawn failed (${inst.rendererId}): ${e.message}`);
            return null;
        }
    }
    warn(msg) {
        (this.deps.warn ?? ((m) => console.warn(`[SimProcessManager] ${m}`)))(msg);
    }
    /** 注册子进程退出监听:非 stop 上下文退出 → error。 */
    watchExit(proc, errorCode) {
        proc.onExit((code) => {
            if (this.stopRequested)
                return;
            if (this.state.status === 'idle' || this.state.status === 'stopping')
                return;
            if (code === 0) {
                if (this.renderScheduler) {
                    this.renderScheduler.stop().catch(() => { });
                    this.renderScheduler = null;
                }
                this.killUeProcs().catch(() => { });
                this.competitionProc = null;
                this.cleanupReady(); // 036: 清理 ready 订阅
                this.state = { status: 'idle', scenario: null, sessionId: null, error: null };
                this.deps.onStateChange(this.getState());
                return;
            }
            this.setState('error', errorCode);
            this.cleanupReady(); // 036: 清理 ready 订阅
            // Spec 028: competition 崩溃时连带停 scheduler + kill UE(避免孤儿)。
            if (this.renderScheduler) {
                this.renderScheduler.stop().catch(() => { });
                this.renderScheduler = null;
            }
            this.killUeProcs().catch(() => { });
            this.killProc().catch(() => { });
        });
    }
    /** 暂停:发 pause 命令给引擎(competition 主循环空转检测自动跟随)。 */
    async pause() {
        if (this.state.status !== 'running')
            throw new Error('not_running');
        await this.deps.redis.publish(this.deps.commandChannel, JSON.stringify({ cmd: 'pause' }));
        this.setState('paused');
        return this.getState();
    }
    async resume() {
        if (this.state.status !== 'paused')
            throw new Error('not_paused');
        await this.deps.redis.publish(this.deps.commandChannel, JSON.stringify({ cmd: 'resume' }));
        this.setState('running');
        return this.getState();
    }
    /** 关闭:发 end → 停 RenderScheduler → 两段式 kill UE → kill competition → 回 idle。幂等。 */
    async stop() {
        if (this.state.status === 'idle')
            return this.getState();
        this.stopRequested = true;
        this.setState('stopping');
        this.cleanupReady(); // 036: 停止后不再等待 ready
        try {
            await this.deps.redis.publish(this.deps.commandChannel, JSON.stringify({ cmd: 'end' }));
        }
        catch {
            // Redis 不可达也要尽力终止进程
        }
        // Spec 028: 先停 RenderScheduler(unsubscribe sim:render_id),再 kill UE 进程。
        if (this.renderScheduler) {
            await this.renderScheduler.stop().catch(() => { });
            this.renderScheduler = null;
        }
        await this.killUeProcs();
        await this.killProc();
        this.competitionProc = null;
        this.stopRequested = false;
        this.startedByBridge = false;
        this.state = { status: 'idle', scenario: null, sessionId: null, error: null };
        this.deps.onStateChange(this.getState());
        return this.getState();
    }
    /** 停止订阅 sim:state 频道(bridge 停止时调用)。 */
    async unsubscribe() {
        if (this.unsubscribeStateChannel) {
            this.unsubscribeStateChannel();
            this.unsubscribeStateChannel = null;
        }
    }
    /** Spec 028: 两段式 kill 所有 UE 进程(SIGTERM → stopGrace → SIGKILL)。 */
    async killUeProcs() {
        const procs = this.ueProcs.filter((p) => !p.exited);
        this.ueProcs = [];
        // 先全部 SIGTERM
        for (const p of procs) {
            try {
                p.kill('SIGTERM');
            }
            catch { /* ignore */ }
        }
        if (procs.some((p) => !p.exited)) {
            await this.deps.sleep(this.deps.stopGrace * 1000);
        }
        // 仍未退出的 SIGKILL
        for (const p of procs) {
            if (!p.exited) {
                try {
                    p.kill('SIGKILL');
                }
                catch { /* ignore */ }
            }
        }
        if (procs.some((p) => !p.exited)) {
            await this.deps.sleep(200);
        }
    }
    /** 两段式:SIGTERM → stopGrace → SIGKILL。 */
    async killProc() {
        const proc = this.competitionProc;
        if (!proc || proc.exited)
            return;
        try {
            proc.kill('SIGTERM');
        }
        catch { /* ignore */ }
        await this.deps.sleep(this.deps.stopGrace * 1000);
        if (!proc.exited) {
            try {
                proc.kill('SIGKILL');
            }
            catch { /* ignore */ }
            await this.deps.sleep(200);
        }
    }
}
exports.SimProcessManager = SimProcessManager;
// ── 生产装配 ──────────────────────────────────────────────────────────
/** 包装 child_process.spawn 为 ManagedProcess。 */
function spawnChildProcess(cmd, args, opts) {
    let stdio;
    if (opts?.logFile) {
        fs.mkdirSync(path.dirname(opts.logFile), { recursive: true });
        const fd = fs.openSync(opts.logFile, 'a');
        stdio = ['ignore', fd, fd];
    }
    else {
        stdio = ['ignore', 'ignore', 'ignore'];
    }
    const child = (0, child_process_1.spawn)(cmd, args, {
        stdio,
        windowsHide: true,
        cwd: opts?.cwd,
        env: opts?.env,
        detached: opts?.detached,
    });
    if (opts?.detached) {
        try {
            child.unref();
        }
        catch { /* ignore */ }
    }
    let exited = false;
    let exitCode = null;
    const exitCbs = [];
    child.on('exit', (code) => {
        exited = true;
        exitCode = code;
        for (const cb of exitCbs)
            cb(code);
    });
    child.on('error', (err) => {
        console.warn(`[spawnChildProcess] spawn error for ${cmd}: ${err.message}`);
        exited = true;
        exitCode = -1;
        for (const cb of exitCbs)
            cb(exitCode);
    });
    return {
        get pid() { return child.pid ?? -1; },
        get exited() { return exited; },
        onExit(cb) {
            if (exited)
                cb(exitCode);
            else
                exitCbs.push(cb);
        },
        kill(signal) {
            if (!child.pid)
                return false;
            if (process.platform === 'win32') {
                try {
                    (0, child_process_1.spawn)('taskkill', ['/PID', String(child.pid), '/T', '/F'], { stdio: 'ignore' });
                    return true;
                }
                catch { /* fall through */ }
            }
            if (opts?.detached) {
                try {
                    process.kill(-child.pid, signal);
                    return true;
                }
                catch { /* fall through */ }
            }
            try {
                return child.kill(signal);
            }
            catch {
                return false;
            }
        },
    };
}
/** 生产环境依赖装配(ioredis publish)。 */
function createProductionDeps(params) {
    const redis = new ioredis_1.default({ host: params.redisHost, port: params.redisPort, lazyConnect: false });
    return {
        spawn: spawnChildProcess,
        redis: {
            async publish(channel, message) { await redis.publish(channel, message); },
            // Spec 028: RenderScheduler 订阅 sim:render_id 用。返回 unsubscribe。
            async subscribe(channel, onMessage) {
                const sub = new ioredis_1.default({ host: params.redisHost, port: params.redisPort, lazyConnect: false });
                await sub.subscribe(channel);
                sub.on('message', (_ch, msg) => onMessage(msg));
                return async () => {
                    try {
                        await sub.unsubscribe(channel);
                    }
                    catch { /* ignore */ }
                    try {
                        sub.disconnect();
                    }
                    catch { /* ignore */ }
                };
            },
        },
        // Spec 028: opensim-render-ctl plan 调用。
        execFile: (0, render_ctl_client_1.createNodeExecFile)(),
        redisHost: params.redisHost,
        redisPort: params.redisPort,
        stopGrace: params.stopGrace,
        commandChannel: params.commandChannel,
        stateChannel: params.stateChannel,
        onStateChange: params.onStateChange,
        sleep: (ms) => new Promise((r) => setTimeout(r, ms)),
        now: () => Date.now(),
        makeSessionId: () => `sess_${Date.now().toString(36)}_${Math.floor(Math.random() * 1e6).toString(36)}`,
    };
}
