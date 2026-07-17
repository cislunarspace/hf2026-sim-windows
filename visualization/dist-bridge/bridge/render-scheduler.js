"use strict";
// Spec 028: RenderScheduler —— bridge 侧的 UE 渲染调度集成层。
//
// 职责(契约见 docs/ue-renderer-integration-requirements.md §3):
//   1. 订阅 sim:render_id,接收 UE 的 renderer_online/renderer_offline 宣告。
//   2. 把待渲染飞机(gimbal UAVs)按各 UE 容量贪心分配,publish assign 指令。
//   3. UE 崩溃(进程退出,由 watchExit 触发)→ 收回其飞机,重分配给其他在线 UE。
//   4. publish stop 指令(迁移/收回场景)。
//
// 数据流分离:UE 始终自取 sim:state 拿全量飞机状态,本模块只管"开关哪架"。
//
// 容量来源:优先用 render-ctl plan 提供的 max_aircraft(capacityOverride),
// 回退到 UE 上报的 max_aircraft。UE 端 max_aircraft 常硬编码(实测上报 2),
// 与 registry 配置不一致会导致第 N 架飞机留 pending 池无人渲染,故以 plan 值为准。
Object.defineProperty(exports, "__esModule", { value: true });
exports.RenderScheduler = void 0;
exports.planAssignment = planAssignment;
const constants_1 = require("../rendering/constants");
// ── 纯逻辑:贪心分配(便于单测) ──────────────────────────────────────────
/**
 * 把 aircraft 按 renderers 容量贪心分配(填满第一个再溢出到下一个)。
 * 不变性:每架飞机至多出现在一个 UE 的分配里。
 */
function planAssignment(aircraft, renderers) {
    const result = new Map();
    let idx = 0;
    for (const r of renderers) {
        const bucket = [];
        while (idx < aircraft.length && bucket.length < r.maxAircraft) {
            bucket.push(aircraft[idx]); // idx < length 已由 while 条件保证
            idx++;
        }
        if (bucket.length > 0)
            result.set(r.renderId, bucket);
    }
    return result;
}
// ── RenderScheduler 类 ────────────────────────────────────────────────
/**
 * 有状态分配器:跟踪 pending 飞机池 + 各 UE 容量 + renderId↔pid 映射。
 *
 * 事件流:
 *   start(gimbalUavs)      → 飞机入 pending 池 + 订阅 sim:render_id
 *   UE online              → addRenderer + 分配 pending → publish assign
 *   UE offline / crash     → removeRenderer → 收回飞机 → 重分配 → publish assign
 *   stop()                 → unsubscribe + 清空状态(不主动发 stop —— UE 进程被 kill)
 */
class RenderScheduler {
    constructor(deps) {
        /** renderId → 容量信息(在线 UE)。 */
        this.renderers = new Map();
        /** pid → renderId(UE 进程崩溃时反查)。 */
        this.pidToRenderId = new Map();
        /** 待分配飞机(无 UE 承接或被收回)。 */
        this.pending = [];
        /** 已分配:renderId → aircraft[](stop/重分配时增量操作)。 */
        this.assigned = new Map();
        /** 超出 UE 渲染容量、需要发 stop 指令的 aircraft ID。 */
        this.excess = new Set();
        /** subscribe 返回的 unsubscribe 句柄。 */
        this.unsub = null;
        this.started = false;
        this.deps = deps;
    }
    /** 启动:记录待渲染飞机池,订阅 sim:render_id。等 UE 上线后自动分配。 */
    async start(aircraft, excessUavs) {
        if (this.started)
            return;
        this.started = true;
        this.pending = [...aircraft];
        this.excess = new Set(excessUavs ?? []);
        this.unsub = await this.deps.subscribe(constants_1.CHANNELS.renderId, async (msg) => {
            try {
                await this.onMessage(msg);
            }
            catch (e) {
                this.warn(`onMessage error: ${e.message}`);
            }
        });
        if (this.pending.length > 0 && this.renderers.size > 0) {
            await this.drainPending();
        }
    }
    /** 注册一个 UE 进程(spawn 时调用,建立 pid↔renderId 映射)。 */
    registerUeProcess(pid, renderId) {
        this.pidToRenderId.set(pid, renderId);
    }
    /** UE 进程崩溃(由 watchExit 触发)。按 pid 反查 renderId 并收回飞机重分配。 */
    async onUeCrash(pid) {
        const renderId = this.pidToRenderId.get(pid);
        if (renderId === undefined)
            return;
        this.pidToRenderId.delete(pid);
        await this.removeRenderer(renderId);
    }
    /** 处理 sim:render_id 频道的消息(renderer_online / renderer_offline)。 */
    async onMessage(raw) {
        let msg;
        try {
            msg = JSON.parse(raw);
        }
        catch {
            this.warn(`ignoring non-JSON render_id message`);
            return;
        }
        if (msg.event === 'renderer_online' && msg.render_id) {
            await this.addRenderer(msg.render_id, typeof msg.max_aircraft === 'number' ? msg.max_aircraft : 0);
        }
        else if (msg.event === 'renderer_offline' && msg.render_id) {
            await this.removeRenderer(msg.render_id);
        }
    }
    /** 停止:unsubscribe + 清空状态。不主动发 stop(UE 进程会被 manager kill)。 */
    async stop() {
        if (!this.started)
            return;
        this.started = false;
        if (this.unsub) {
            try {
                await this.unsub();
            }
            catch { /* best-effort */ }
            this.unsub = null;
        }
        this.renderers.clear();
        this.pidToRenderId.clear();
        this.assigned.clear();
        this.pending = [];
    }
    // ── 内部 ──────────────────────────────────────────────────────────
    /** UE 上线:记录容量(优先 capacityOverride),分配 pending 飞机,对超出容量的 aircraft 发 stop。 */
    async addRenderer(renderId, reported) {
        if (this.renderers.has(renderId))
            return; // 幂等:重复 online 忽略
        const cap = this.deps.capacityOverride?.[renderId] ?? reported;
        const maxAircraft = cap > 0 ? cap : reported;
        if (maxAircraft <= 0) {
            this.warn(`renderer ${renderId} online with maxAircraft<=0 (reported=${reported}, override=${this.deps.capacityOverride?.[renderId]}); skipping`);
            return;
        }
        this.renderers.set(renderId, { renderId, maxAircraft });
        this.assigned.set(renderId, new Set());
        this.info(`renderer online: ${renderId} maxAircraft=${maxAircraft}`);
        // 第一个 UE 上线时,把超出容量的 aircraft 发 stop 指令,明确告诉 UE 不要渲染。
        if (this.excess.size > 0 && this.renderers.size === 1) {
            const ac = [...this.excess];
            this.info(`stop excess aircraft: ${ac.join(',')}`);
            await this.publish({ event: 'stop', render_id: renderId, aircraft: ac });
        }
        await this.drainPending();
    }
    /** UE 下线/崩溃:收回其飞机入 pending,重分配给其他 UE,publish stop+assign。 */
    async removeRenderer(renderId) {
        const info = this.renderers.get(renderId);
        if (!info)
            return;
        const taken = this.assigned.get(renderId);
        this.renderers.delete(renderId);
        this.assigned.delete(renderId);
        if (taken && taken.size > 0) {
            // 通知该 UE 停止这些飞机(尽力;UE 可能已死,redis publish 即返回)。
            const ac = [...taken];
            await this.publish({ event: 'stop', render_id: renderId, aircraft: ac });
            this.pending.push(...ac);
        }
        await this.drainPending();
    }
    /** 把 pending 飞机按各 UE 容量贪心分配,publish assign 指令。 */
    async drainPending() {
        if (this.pending.length === 0 || this.renderers.size === 0)
            return;
        const renderers = [...this.renderers.values()].sort((a, b) => a.renderId.localeCompare(b.renderId)); // 确定性顺序
        // 各 UE 剩余容量 = maxAircraft - 已分配数。
        const remaining = new Map();
        for (const r of renderers) {
            remaining.set(r.renderId, r.maxAircraft - (this.assigned.get(r.renderId)?.size ?? 0));
        }
        const stillPending = [];
        for (const ac of this.pending) {
            // 找第一个有剩余容量的 UE(确定性,按 renderId 序)。
            const target = renderers.find((r) => (remaining.get(r.renderId) ?? 0) > 0);
            if (!target) {
                stillPending.push(ac);
                continue;
            }
            this.assigned.get(target.renderId).add(ac);
            remaining.set(target.renderId, remaining.get(target.renderId) - 1);
        }
        this.pending = stillPending;
        // 合并:把本次新分配到同一 UE 的飞机合成一条 assign 指令(减少消息数)。
        const newlyAssigned = new Map();
        for (const r of renderers) {
            const all = this.assigned.get(r.renderId);
            if (!all)
                continue;
            // 只发"本次 drain 新增的"是复杂的(需 diff);简化:UE 端 assign 幂等,
            // 重发全量是安全的(契约 §7:重复 assign 同一架飞机 = 幂等无操作)。
            if (all.size > 0)
                newlyAssigned.set(r.renderId, [...all]);
        }
        for (const [rid, ac] of newlyAssigned) {
            this.info(`assign renderer=${rid} aircraft=${ac.join(',')}`);
            await this.publish({ event: 'assign', render_id: rid, aircraft: ac });
        }
    }
    publish(cmd) {
        return this.deps.publish(constants_1.CHANNELS.renderId, JSON.stringify(cmd));
    }
    warn(msg) {
        (this.deps.warn ?? ((m) => console.warn(`[RenderScheduler] ${m}`)))(msg);
    }
    info(msg) {
        (this.deps.info ?? ((m) => console.log(`[RenderScheduler] ${m}`)))(msg);
    }
}
exports.RenderScheduler = RenderScheduler;
